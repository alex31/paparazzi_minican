/**
 * @file imavRole.cpp
 * @brief IMAV beacon sound and light detection role.
 */

#include "roleConf.h"

#if USE_IMAV_ROLE

#include "imavRole.hpp"
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"
#include "resourceManager.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"

#include <algorithm>
#include <array>

namespace {
  constexpr size_t audioBufferDepth = 1024U;
  constexpr size_t audioHalfDepth = audioBufferDepth / 2U;
  constexpr gptcnt_t audioTimerInterval = 1771U;
  constexpr sysinterval_t healthPeriod = TIME_MS2I(1000U);
  constexpr sysinterval_t opticalRetryPeriod = TIME_MS2I(5000U);
  constexpr uint16_t opt4048DeviceIdMask = 0xFFF0U;
  constexpr uint16_t opt4048DeviceId = 0x0820U;
  constexpr uint8_t opt4048RegChannels = 0x00U;
  constexpr uint8_t opt4048RegConfig = 0x0AU;
  constexpr uint8_t opt4048RegInterrupt = 0x0BU;
  constexpr uint8_t opt4048RegStatus = 0x0CU;
  constexpr uint8_t opt4048RegDeviceId = 0x11U;
  constexpr uint16_t opt4048ConfigContinuous1ms =
    (12U << 10U) |  // Automatic full-scale range.
    (1U << 6U) |    // 1 ms conversion time per channel.
    (3U << 4U);     // Continuous four-channel conversion.
  constexpr uint16_t opt4048InterruptBurst = 0x8011U;
  constexpr uint16_t opt4048StatusOverload = 1U << 3U;
  constexpr uint8_t opticalFrameAttempts = 4U;

  struct GoertzelBin {
    float coefficient;
    uint16_t frequencyHz;
  };

  // Fs = 42.5 MHz / 1771 = 23997.741389 Hz. The four outer bins estimate
  // nearby noise; the eleven inner bins cover the rulebook's 2.6--3.0 kHz
  // alarm range with a small tolerance on either side.
  constexpr std::array<GoertzelBin, 15U> goertzelBins = {{
    {1.662877621f, 2250U},
    {1.633216270f, 2350U},
    {1.570556061f, 2550U},
    {1.554211286f, 2600U},
    {1.537600155f, 2650U},
    {1.520725515f, 2700U},
    {1.503590257f, 2750U},
    {1.486197318f, 2800U},
    {1.468549679f, 2850U},
    {1.450650364f, 2900U},
    {1.432502441f, 2950U},
    {1.414109020f, 3000U},
    {1.395473253f, 3050U},
    {1.318571212f, 3250U},
    {1.258510566f, 3400U},
  }};
  constexpr size_t firstAlarmBin = 2U;
  constexpr size_t lastAlarmBin = 12U;
  constexpr size_t alarmBinCount = lastAlarmBin - firstAlarmBin + 1U;
  constexpr float hannCosStep = 0.99992440685f;
  constexpr float hannSinStep = 0.01229555183f;
  constexpr uint16_t audioClipLow = 82U;
  constexpr uint16_t audioClipHigh = 8108U;
  constexpr float minimumSpectralRatio = 1.0e-6f;

  constexpr eventmask_t audioReadyEvent = EVENT_MASK(0);
  constexpr eventmask_t audioErrorEvent = EVENT_MASK(1);
  constexpr eventmask_t allAudioEvents = audioReadyEvent | audioErrorEvent;

  constexpr uint32_t oversamplingX4Keep13Bits =
    ADC_CFGR2_ROVSE |
    (1U << ADC_CFGR2_OVSR_Pos) |
    (1U << ADC_CFGR2_OVSS_Pos);

  constexpr GPTConfig audioTimerConfig = {
    .frequency = 42'500'000U,
    .callback = nullptr,
    .cr2 = TIM_CR2_MMS_1,
    .dier = 0U,
  };
}

struct AudioChannelScore {
  float bandPower = 0.0f;
  float peakPower = 0.0f;
  float referencePower = 0.0f;
  float noiseFloor = 0.0f;
  float concentration = 0.0f;
  float spectralRatioDb = -60.0f;
  float prominence = 0.0f;
  float blockScore = 0.0f;
  float toneRms = 0.0f;
  uint16_t peakFrequencyHz = 0U;
  uint16_t minimum = 0U;
  uint16_t maximum = 0U;
  uint16_t clippedSamples = 0U;
  bool floorValid = false;
};

enum class AudioBurstState : uint8_t {
  Unarmed,
  Off,
  On,
};

struct AudioDetector {
  float q1[goertzelBins.size()] = {};
  float q2[goertzelBins.size()] = {};
  float power[goertzelBins.size()] = {};
  AudioChannelScore channel;
  float dominantFrequencyHz = 0.0f;
  float blockScore = 0.0f;
  float burstPeakScore = 0.0f;
  float recentBurstStrength = 0.0f;
  float cadenceHz = 0.0f;
  float cadenceScore = 0.0f;
  float audioScore = 0.0f;
  systime_t onsets[6] = {};
  systime_t lastBlockTime = 0U;
  systime_t lastOnsetTime = 0U;
  uint8_t onsetCount = 0U;
  uint8_t highBlocks = 0U;
  uint8_t lowBlocks = 0U;
  AudioBurstState burstState = AudioBurstState::Unarmed;
  bool detected = false;
};

/** @brief DMA-backed state shared by the ADC callback and worker. */
struct ImavAudioState {
  adcsample_t samples[audioBufferDepth];
  const ADCConversionGroup *adcGroup = nullptr;
  thread_t *worker = nullptr;
  thread_t *opticalWorker = nullptr;
  volatile uint32_t adcSequence = 0U;
  volatile adcerror_t errors = 0U;
  uint32_t processedBlocks = 0U;
  uint32_t droppedBlocks = 0U;
  uint32_t stalledBlocks = 0U;
  uint32_t acquisitionRestarts = 0U;
  uint32_t discontinuities = 0U;
  uint16_t mean = 0U;
  uint16_t meanAbsoluteDeviation = 0U;
  AudioDetector detector;
  uint8_t opticalAddress = 0x44U;
  uint8_t opticalTx[3] = {};
  uint8_t opticalRx[16] = {};
  uint16_t opticalDeviceId = 0U;
  uint16_t opticalStatus = 0U;
  bool opticalAvailable = false;
  bool opticalBackgroundValid = false;
  uint8_t opticalCounter[4] = {};
  uint32_t opticalRaw[4] = {};
  uint32_t opticalBackground[4] = {};
  float opticalRelativeAc[4] = {};
  float opticalNoiseFloor = 0.0f;
  float opticalFlashHold = 0.0f;
  float opticalFlashScore = 0.0f;
  uint32_t opticalPositiveAc = 0U;
  uint32_t opticalPulseCount = 0U;
  uint32_t opticalValidFrames = 0U;
  systime_t opticalLastValidSample = 0U;
  uint32_t opticalReadErrors = 0U;
  uint32_t opticalBusRecoveries = 0U;
  uint32_t opticalCrcErrors = 0U;
  uint32_t opticalTornFrames = 0U;
  uint8_t opticalConsecutiveTransportErrors = 0U;
  bool opticalNoiseFloorValid = false;
  bool opticalPulseActive = false;
  bool publishDebug = true;
};

static_assert(sizeof(adcsample_t) == sizeof(uint16_t));

namespace {
  /** @brief Linear 0..1 knee used by provisional, measurable DSP scores. */
  float knee(float value, float low, float high)
  {
    return std::clamp((value - low) / (high - low), 0.0f, 1.0f);
  }

  /** @brief Analyze one audio block without retaining samples. */
  void analyzeAudioBlock(ImavAudioState& state, size_t offset,
			 uint16_t mean)
  {
    AudioDetector& detector = state.detector;
    for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
      detector.q1[bin] = 0.0f;
      detector.q2[bin] = 0.0f;
    }

    float windowedEnergy = 0.0f;
    uint16_t minimum = UINT16_MAX;
    uint16_t maximum = 0U;
    uint16_t clipped = 0U;
    float cosine = 1.0f;
    float sine = 0.0f;

    for (size_t index = 0U; index < audioHalfDepth; ++index) {
      const uint16_t raw = state.samples[offset + index];
      const float window = 0.5f * (1.0f - cosine);
      const float sample = (static_cast<float>(raw) - mean) * window;

      minimum = std::min(minimum, raw);
      maximum = std::max(maximum, raw);
      if ((raw <= audioClipLow) || (raw >= audioClipHigh)) {
	++clipped;
      }
      windowedEnergy += sample * sample;

      for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
	const float q0 = sample +
	  goertzelBins[bin].coefficient * detector.q1[bin] -
	  detector.q2[bin];
	detector.q2[bin] = detector.q1[bin];
	detector.q1[bin] = q0;
      }

      const float nextCosine = cosine * hannCosStep - sine * hannSinStep;
      sine = sine * hannCosStep + cosine * hannSinStep;
      cosine = nextCosine;
    }

    AudioChannelScore& score = detector.channel;
    float bandPower = 0.0f;
    float peakPower = 0.0f;
    size_t peakBin = firstAlarmBin;

    for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
      const float q1 = detector.q1[bin];
      const float q2 = detector.q2[bin];
      const float rawPower = q1 * q1 + q2 * q2 -
	goertzelBins[bin].coefficient * q1 * q2;
      const float power = std::max(rawPower, 0.0f);
      detector.power[bin] = power;
      if ((bin >= firstAlarmBin) && (bin <= lastAlarmBin)) {
	bandPower += power;
	if (power > peakPower) {
	  peakPower = power;
	  peakBin = bin;
	}
      }
    }

    const float referencePower =
      (detector.power[0] + detector.power[1] +
       detector.power[13] + detector.power[14]) * 0.25f;
    bandPower /= static_cast<float>(alarmBinCount);

    if (not score.floorValid) {
      score.noiseFloor = bandPower;
      score.floorValid = true;
    } else if (detector.burstState != AudioBurstState::On) {
      const float alpha = bandPower < score.noiseFloor ? 1.0f / 16.0f
						    : 1.0f / 256.0f;
      score.noiseFloor += (bandPower - score.noiseFloor) * alpha;
    }

    score.bandPower = bandPower;
    score.peakPower = peakPower;
    score.referencePower = referencePower;
    // The factor 3 compensates the Hann window's coherent/power gains, so a
    // pure tone centered on a candidate frequency approaches 0 dB.
    score.concentration = std::clamp(
      3.0f * peakPower /
	(static_cast<float>(audioHalfDepth) * windowedEnergy + 1.0f),
      0.0f, 1.0f);
    score.spectralRatioDb = 10.0f * __builtin_log10f(
      std::max(score.concentration, minimumSpectralRatio));
    score.prominence = bandPower / (referencePower + 1.0f);
    // Hann coherent gain gives A_rms ~= sqrt(Goertzel power) * sqrt(8) / N.
    score.toneRms = __builtin_sqrtf(peakPower) * 0.005524272f;
    const float prominenceScore = knee(score.prominence, 1.5f, 5.0f);
    const float riseScore = knee(
      bandPower / (score.noiseFloor + 1.0f), 1.8f, 6.0f);
    const float tonalityScore = knee(score.spectralRatioDb, -15.0f, -6.0f);
    score.blockScore = tonalityScore *
      std::max(prominenceScore, 0.8f * riseScore);
    if (clipped >= 5U) {
      score.blockScore *= 0.8f;
    }
    score.peakFrequencyHz = goertzelBins[peakBin].frequencyHz;
    score.minimum = minimum;
    score.maximum = maximum;
    score.clippedSamples = clipped;

    float strongestPower = 0.0f;
    size_t strongestBin = firstAlarmBin;
    for (size_t bin = firstAlarmBin; bin <= lastAlarmBin; ++bin) {
      if (detector.power[bin] > strongestPower) {
	strongestPower = detector.power[bin];
	strongestBin = bin;
      }
    }

    float binOffset = 0.0f;
    if ((strongestBin > firstAlarmBin) && (strongestBin < lastAlarmBin)) {
      const float left = detector.power[strongestBin - 1U];
      const float center = strongestPower;
      const float right = detector.power[strongestBin + 1U];
      const float denominator = left - 2.0f * center + right;
      if (denominator < -1.0f) {
	binOffset = std::clamp(0.5f * (left - right) / denominator,
			       -0.5f, 0.5f);
      }
    }
    detector.dominantFrequencyHz =
      static_cast<float>(goertzelBins[strongestBin].frequencyHz) +
      50.0f * binOffset;
    detector.blockScore = score.blockScore;
  }

  /** @brief Drop cadence evidence after a long gap without inventing an edge. */
  void clearAudioCadence(AudioDetector& detector)
  {
    detector.onsetCount = 0U;
    detector.lastOnsetTime = 0U;
    detector.cadenceHz = 0.0f;
    detector.cadenceScore = 0.0f;
    detector.audioScore = 0.0f;
    detector.recentBurstStrength = 0.0f;
    detector.burstPeakScore = 0.0f;
    detector.highBlocks = 0U;
    detector.lowBlocks = 0U;
    detector.burstState = AudioBurstState::Unarmed;
    detector.detected = false;
  }

  /** @brief Append an onset to the fixed six-entry chronological history. */
  void appendAudioOnset(AudioDetector& detector, systime_t now)
  {
    if ((detector.lastOnsetTime != 0U) &&
	(chTimeDiffX(detector.lastOnsetTime, now) < TIME_MS2I(120U))) {
      return;
    }

    if (detector.onsetCount < std::size(detector.onsets)) {
      detector.onsets[detector.onsetCount++] = now;
    } else {
      for (size_t index = 1U; index < std::size(detector.onsets); ++index) {
	detector.onsets[index - 1U] = detector.onsets[index];
      }
      detector.onsets[std::size(detector.onsets) - 1U] = now;
    }
    detector.lastOnsetTime = now;
  }

  /** @brief Extract alarm bursts and compare their cadence to about 3 Hz. */
  void updateAudioCadence(AudioDetector& detector, bool discontinuity,
			  systime_t now)
  {
    if ((detector.lastBlockTime != 0U) &&
	(chTimeDiffX(detector.lastBlockTime, now) >= TIME_MS2I(200U))) {
      clearAudioCadence(detector);
    }
    detector.lastBlockTime = now;

    if (discontinuity) {
      detector.burstState = AudioBurstState::Unarmed;
      detector.highBlocks = 0U;
      detector.lowBlocks = 0U;
      detector.burstPeakScore = 0.0f;
    }

    const bool high = detector.blockScore >= 0.62f;
    const bool low = detector.blockScore <= 0.30f;
    switch (detector.burstState) {
    case AudioBurstState::Unarmed:
      detector.highBlocks = 0U;
      detector.lowBlocks = low ? static_cast<uint8_t>(detector.lowBlocks + 1U)
			       : 0U;
      if (detector.lowBlocks >= 2U) {
	detector.burstState = AudioBurstState::Off;
	detector.lowBlocks = 0U;
      }
      break;

    case AudioBurstState::Off:
      detector.lowBlocks = 0U;
      detector.highBlocks = high ? static_cast<uint8_t>(detector.highBlocks + 1U)
				 : 0U;
      if (detector.highBlocks >= 2U) {
	detector.burstState = AudioBurstState::On;
	detector.highBlocks = 0U;
	detector.burstPeakScore = detector.blockScore;
	appendAudioOnset(detector, now);
      }
      break;

    case AudioBurstState::On:
      detector.highBlocks = 0U;
      detector.burstPeakScore =
	std::max(detector.burstPeakScore, detector.blockScore);
      detector.lowBlocks = low ? static_cast<uint8_t>(detector.lowBlocks + 1U)
			       : 0U;
      if (detector.lowBlocks >= 2U) {
	detector.burstState = AudioBurstState::Off;
	detector.lowBlocks = 0U;
	detector.recentBurstStrength = detector.burstPeakScore;
	detector.burstPeakScore = 0.0f;
      }
      break;
    }

    if (detector.lastOnsetTime == 0U) {
      detector.cadenceHz = 0.0f;
      detector.cadenceScore = 0.0f;
      detector.audioScore = 0.0f;
      return;
    }

    const uint32_t onsetAgeMs = TIME_I2MS(
      chTimeDiffX(detector.lastOnsetTime, now));
    if (onsetAgeMs >= 1500U) {
      clearAudioCadence(detector);
      return;
    }

    float periodScoreSum = 0.0f;
    float periodMsSum = 0.0f;
    const uint8_t intervalCount = detector.onsetCount > 0U
      ? static_cast<uint8_t>(detector.onsetCount - 1U) : 0U;
    for (uint8_t index = 1U; index < detector.onsetCount; ++index) {
      const float periodMs = static_cast<float>(TIME_I2MS(
	chTimeDiffX(detector.onsets[index - 1U], detector.onsets[index])));
      const float error = (periodMs - 333.333f) / 80.0f;
      periodScoreSum += 1.0f / (1.0f + error * error);
      periodMsSum += periodMs;
    }

    if (intervalCount == 0U) {
      detector.cadenceHz = 0.0f;
      detector.cadenceScore = 0.0f;
      detector.audioScore = 0.0f;
      return;
    }

    const float meanPeriodMs = periodMsSum / intervalCount;
    detector.cadenceHz = 1000.0f / meanPeriodMs;
    const float support = std::min(static_cast<float>(intervalCount) / 3.0f,
				   1.0f);
    const float freshness = onsetAgeMs <= 450U ? 1.0f :
      std::max(0.0f, (900.0f - static_cast<float>(onsetAgeMs)) / 450.0f);
    detector.cadenceScore =
      (periodScoreSum / intervalCount) * support * freshness;
    const float strength = detector.burstState == AudioBurstState::On
      ? std::max(detector.recentBurstStrength, detector.burstPeakScore)
      : detector.recentBurstStrength;
    detector.audioScore = detector.cadenceScore * (0.5f + 0.5f * strength);
    if (detector.detected) {
      detector.detected = detector.audioScore >= 0.35f;
    } else {
      detector.detected = detector.audioScore >= 0.65f;
    }
  }
}

DeviceStatus ImavRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  return DeviceStatus(DeviceStatus::IMAV_ROLE);
}

DeviceStatus ImavRole::start(UAVCAN::Node& node)
{
  m_node = &node;

  using HR = HWResource;
  if (not boardResource.tryAcquire(HR::PA03, HR::ADC_1, HR::TIM_6)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::ADC_1));
  }

  if (ADCD1.state != ADC_READY) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::ADC_1));
  }

  DeviceStatus status(DeviceStatus::IMAV_ROLE);
  audio = try_new_dma<ImavAudioState>(DeviceStatus::IMAV_ROLE, status);
  if (not status) {
    return status;
  }

  static const ADCConversionGroup adcAudioGroup = {
    .circular = true,
    .num_channels = 1U,
    .end_cb = &Trampoline<&ImavRole::audioDmaCallback>::fn,
    .error_cb = &Trampoline<&ImavRole::audioErrorCallback>::fn,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_EXTSEL_SRC(13U) |
	    ADC_CFGR_EXTEN_RISING,
    .cfgr2 = oversamplingX4Keep13Bits,
    .tr1 = ADC_TR_DISABLED,
    .tr2 = ADC_TR_DISABLED,
    .tr3 = ADC_TR_DISABLED,
    .awd2cr = 0U,
    .awd3cr = 0U,
    .smpr = {
      ADC_SMPR1_SMP_AN4(ADC_SMPR_SMP_47P5),
      0U,
    },
    .sqr = {
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4),
      0U,
      0U,
      0U,
    },
  };

  audio->adcGroup = &adcAudioGroup;
  audio->opticalAddress = static_cast<uint8_t>(
    param_cget<"role.imav.light.i2c_address">());
  audio->publishDebug = param_cget<"role.imav.debug.publish">();

  // OPT4048 supports Standard/Fast mode up to 400 kHz. Its advertised
  // 2.6 MHz mode requires an I2C High-Speed controller code that this driver
  // does not emit; Fast-mode Plus at 1 MHz is therefore not interchangeable.
  const uint32_t i2cFrequencyKhz = param_cget<"bus.i2c.frequency_khz">();
  if (i2cFrequencyKhz >= 1000U) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE,
			DeviceStatus::I2C_FREQ_INVALID,
			static_cast<uint16_t>(i2cFrequencyKhz));
  }

  const DeviceStatus i2cStatus = I2CPeriph::start();
  if (not i2cStatus) {
    return i2cStatus;
  }
  audio->opticalAvailable = initializeOpticalSensor();
  const bool opticalInitiallyAvailable = audio->opticalAvailable;
  const uint16_t initialOpticalDeviceId = audio->opticalDeviceId;

  palSetLineMode(LINE_DBG_RX, PAL_MODE_INPUT_ANALOG);
  if (gptStart(&GPTD6, &audioTimerConfig) != MSG_OK) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE, DeviceStatus::NOT_RESPONDING,
			std::to_underlying(HR::TIM_6));
  }

  audio->worker = chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(1536U),
				      "imav audio", NORMALPRIO,
				      &Trampoline<&ImavRole::audioThread>::fn,
				      this);
  if (audio->worker == nullptr) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE, DeviceStatus::HEAP_FULL);
  }

  startAudioAcquisition();
  audio->opticalWorker = chThdCreateFromHeap(
    nullptr, THD_WORKING_AREA_SIZE(768U), "imav light", NORMALPRIO - 1,
    &Trampoline<&ImavRole::opticalThread>::fn, this);
  if (audio->opticalWorker == nullptr) {
    audio->opticalAvailable = false;
    node.infoCb("IMAV light worker unavailable: heap full");
  }

  node.infoCb("IMAV audio started: PA3/ADC1, 24kHz, OVS x4");
  if (opticalInitiallyAvailable) {
    node.infoCb("IMAV light started: OPT4048 addr=0x%02x id=0x%04x",
		audio->opticalAddress, initialOpticalDeviceId);
  } else {
    node.infoCb("IMAV light unavailable: OPT4048 addr=0x%02x",
		audio->opticalAddress);
  }
  return DeviceStatus(DeviceStatus::IMAV_ROLE);
}

/** @brief Start the ADC DMA stream before enabling its timer trigger. */
void ImavRole::startAudioAcquisition()
{
  chSysLock();
  audio->adcSequence = 0U;
  audio->errors = 0U;
  chSysUnlock();

  adcStartConversion(&ADCD1, audio->adcGroup, audio->samples,
		     audioBufferDepth);
  gptStartContinuous(&GPTD6, audioTimerInterval);
}

/** @brief Stop the trigger first, then return ADC1 to READY state. */
void ImavRole::stopAudioAcquisition()
{
  gptStopTimer(&GPTD6);
  adcStopConversion(&ADCD1);
}

/** @brief Minimal ISR callback: count half-buffers and wake the worker. */
void ImavRole::audioDmaCallback(ADCDriver *adcp)
{
  chSysLockFromISR();
  if (adcp == &ADCD1) {
    ++audio->adcSequence;
  }
  if (audio->worker != nullptr) {
    chEvtSignalI(audio->worker, audioReadyEvent);
  }
  chSysUnlockFromISR();
}

/** @brief Record an ADC error and defer recovery to thread context. */
void ImavRole::audioErrorCallback(ADCDriver *, adcerror_t error)
{
  chSysLockFromISR();
  audio->errors |= error;
  if (audio->worker != nullptr) {
    chEvtSignalI(audio->worker, audioErrorEvent);
  }
  chSysUnlockFromISR();
}

/** @brief Compute cheap bring-up statistics without copying the DMA block. */
void ImavRole::processAudioHalf(size_t offset, bool discontinuity)
{
  uint32_t sum = 0U;
  for (size_t i = 0U; i < audioHalfDepth; ++i) {
    sum += audio->samples[offset + i];
  }

  const uint16_t mean = static_cast<uint16_t>(sum / audioHalfDepth);
  uint32_t deviation = 0U;
  for (size_t i = 0U; i < audioHalfDepth; ++i) {
    const uint16_t sample = audio->samples[offset + i];
    deviation += sample >= mean ? sample - mean : mean - sample;
  }

  audio->mean = mean;
  audio->meanAbsoluteDeviation =
    static_cast<uint16_t>(deviation / audioHalfDepth);
  analyzeAudioBlock(*audio, offset, mean);
  updateAudioCadence(audio->detector, discontinuity, chVTGetSystemTimeX());
  ++audio->processedBlocks;
  if (discontinuity) {
    ++audio->discontinuities;
  }

  if ((audio->processedBlocks % 47U) == 0U) {
    uint32_t opticalPositiveAc;
    uint16_t opticalStatus;
    float opticalFlashScore;
    systime_t opticalLastValidSample;
    chSysLock();
    opticalPositiveAc = audio->opticalPositiveAc;
    opticalStatus = audio->opticalStatus;
    opticalFlashScore = audio->opticalFlashScore;
    opticalLastValidSample = audio->opticalLastValidSample;
    chSysUnlock();

    const systime_t now = chVTGetSystemTimeX();
    const bool opticalFresh = (opticalLastValidSample != 0U) &&
      (chTimeDiffX(opticalLastValidSample, now) < TIME_MS2I(200U));
    const uint16_t lightScore = opticalFresh
      ? static_cast<uint16_t>(1000.0f * opticalFlashScore) : 0U;

    const uint16_t blockScore = static_cast<uint16_t>(
	1000.0f * audio->detector.channel.blockScore);
    const uint16_t frequency = static_cast<uint16_t>(
      audio->detector.dominantFrequencyHz);
    const uint16_t cadence = static_cast<uint16_t>(
      1000.0f * audio->detector.cadenceHz);
    const uint16_t audioScore = static_cast<uint16_t>(
      1000.0f * audio->detector.audioScore);
    m_node->infoCb("IMAV a=%u f=%u c=%u s=%u d=%u",
		   blockScore, frequency, cadence, audioScore,
		   audio->detector.detected ? 1U : 0U);
    const int16_t spectralDb10 = static_cast<int16_t>(
	10.0f * audio->detector.channel.spectralRatioDb);
    m_node->infoCb("IMAV dc=%u mad=%u clip=%u sdb10=%d",
		   audio->mean, audio->meanAbsoluteDeviation,
		   audio->detector.channel.clippedSamples, spectralDb10);
    m_node->infoCb("IMAV drop=%lu gap=%lu light=%lu/%u ovl=%u",
		   audio->droppedBlocks, audio->discontinuities,
		   opticalPositiveAc, lightScore,
		   (opticalStatus & opt4048StatusOverload) != 0U ? 1U : 0U);
    publishDebugValues();
  }
}

/** @brief Publish compact one-frame values for threshold tuning in flight. */
void ImavRole::publishDebugValues()
{
  if (not audio->publishDebug) {
    return;
  }

  float opticalFlashScore;
  systime_t opticalLastValidSample;
  chSysLock();
  opticalFlashScore = audio->opticalFlashScore;
  opticalLastValidSample = audio->opticalLastValidSample;
  chSysUnlock();
  const systime_t now = chVTGetSystemTimeX();
  if ((opticalLastValidSample == 0U) ||
      (chTimeDiffX(opticalLastValidSample, now) >= TIME_MS2I(200U))) {
    opticalFlashScore = 0.0f;
  }

  uavcan_protocol_debug_KeyValue message = {};
  const auto publish = [this, &message](const char *key, float value) {
    message.value = value;
    UAVCAN::dsdlAssign(message.key, key);
    m_node->sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW);
  };
  publish("a0", audio->detector.channel.blockScore);
  publish("p0", audio->detector.channel.toneRms);
  publish("sdb", audio->detector.channel.spectralRatioDb);
  publish("aud", audio->detector.audioScore);
  publish("frq", audio->detector.dominantFrequencyHz);
  publish("cad", audio->detector.cadenceHz);
  publish("lit", opticalFlashScore);
}

/** @brief Execute an OPT4048 transaction using DMA-accessible buffers. */
msg_t ImavRole::opticalTransfer(size_t txLength, size_t rxLength)
{
  i2cAcquireBus(&ExternalI2CD);
  const msg_t result = i2cMasterTransmitTimeout(
    &ExternalI2CD, audio->opticalAddress,
    audio->opticalTx, txLength,
    audio->opticalRx, rxLength,
    TIME_MS2I(10U));

  if (result != MSG_OK) {
    ++audio->opticalReadErrors;
    const i2cflags_t errors = i2cGetErrors(&ExternalI2CD);
    // A missing optional sensor normally returns ACK_FAILURE and must not
    // disturb the other roles on the shared bus. Recover only from a timeout
    // or an actual electrical/protocol fault. Recovery changes the peripheral
    // and pins, so it remains inside the I2C mutex.
    if ((result == MSG_TIMEOUT) ||
	((errors & ~static_cast<i2cflags_t>(I2C_ACK_FAILURE)) != I2C_NO_ERROR)) {
      ++audio->opticalBusRecoveries;
      I2CPeriph::resetLocked();
    }
  }
  i2cReleaseBus(&ExternalI2CD);
  return result;
}

/** @brief Read one big-endian 16-bit OPT4048 register. */
bool ImavRole::readOpticalRegister(uint8_t reg, uint16_t& value)
{
  audio->opticalTx[0] = reg;
  if (opticalTransfer(1U, 2U) != MSG_OK) {
    return false;
  }
  value = static_cast<uint16_t>(audio->opticalRx[0]) << 8U |
          static_cast<uint16_t>(audio->opticalRx[1]);
  return true;
}

/** @brief Write one big-endian 16-bit OPT4048 register. */
bool ImavRole::writeOpticalRegister(uint8_t reg, uint16_t value)
{
  audio->opticalTx[0] = reg;
  audio->opticalTx[1] = static_cast<uint8_t>(value >> 8U);
  audio->opticalTx[2] = static_cast<uint8_t>(value);
  return opticalTransfer(3U, 0U) == MSG_OK;
}

/** @brief Probe and configure the OPT4048 for continuous burst reads. */
bool ImavRole::initializeOpticalSensor()
{
  uint16_t id = 0U;
  if (not readOpticalRegister(opt4048RegDeviceId, id)) {
    return false;
  }
  audio->opticalDeviceId = id;
  // Datasheet revisions and available breakout libraries report 0x0820 and
  // 0x0821. The low nibble is treated as a silicon revision.
  if ((id & opt4048DeviceIdMask) != opt4048DeviceId) {
    return false;
  }
  if (not writeOpticalRegister(opt4048RegInterrupt,
			       opt4048InterruptBurst)) {
    return false;
  }
  if (not writeOpticalRegister(opt4048RegConfig,
			       opt4048ConfigContinuous1ms)) {
    return false;
  }

  chThdSleepMilliseconds(5U);
  audio->opticalBackgroundValid = false;
  audio->opticalNoiseFloorValid = false;
  audio->opticalPulseActive = false;
  audio->opticalFlashHold = 0.0f;
  chSysLock();
  audio->opticalFlashScore = 0.0f;
  audio->opticalLastValidSample = 0U;
  chSysUnlock();
  return true;
}

/** @brief Calculate the four-bit check code described in the OPT4048 data sheet. */
static uint8_t opt4048Crc(uint8_t exponent, uint32_t mantissa,
			  uint8_t counter)
{
  uint8_t x0 = 0U;
  for (uint8_t bit = 0U; bit < 4U; ++bit) {
    x0 ^= static_cast<uint8_t>((exponent >> bit) & 1U);
    x0 ^= static_cast<uint8_t>((counter >> bit) & 1U);
  }
  for (uint8_t bit = 0U; bit < 20U; ++bit) {
    x0 ^= static_cast<uint8_t>((mantissa >> bit) & 1U);
  }

  uint8_t x1 = static_cast<uint8_t>(((counter >> 1U) ^
				     (counter >> 3U) ^
				     (exponent >> 1U) ^
				     (exponent >> 3U)) & 1U);
  for (uint8_t bit = 1U; bit < 20U; bit += 2U) {
    x1 ^= static_cast<uint8_t>((mantissa >> bit) & 1U);
  }

  uint8_t x2 = static_cast<uint8_t>(((counter >> 3U) ^
				     (exponent >> 3U)) & 1U);
  for (uint8_t bit = 3U; bit < 20U; bit += 4U) {
    x2 ^= static_cast<uint8_t>((mantissa >> bit) & 1U);
  }

  const uint8_t x3 = static_cast<uint8_t>(
    ((mantissa >> 3U) ^ (mantissa >> 11U) ^ (mantissa >> 19U)) & 1U);
  return static_cast<uint8_t>((x3 << 3U) | (x2 << 2U) |
			      (x1 << 1U) | x0);
}

/** @brief Read, validate and linearize the four OPT4048 channels. */
bool ImavRole::readOpticalSensor()
{
  uint32_t samples[4] = {};
  uint8_t counters[4] = {};
  bool frameValid = false;

  for (uint8_t attempt = 0U; attempt < opticalFrameAttempts; ++attempt) {
    audio->opticalTx[0] = opt4048RegChannels;
    if (opticalTransfer(1U, sizeof(audio->opticalRx)) != MSG_OK) {
      return false;
    }

    bool crcValid = true;
    for (size_t channel = 0U; channel < 4U; ++channel) {
      const size_t offset = channel * 4U;
      const uint8_t exponent = audio->opticalRx[offset] >> 4U;
      const uint32_t mantissa =
	(static_cast<uint32_t>(audio->opticalRx[offset] & 0x0FU) << 16U) |
	(static_cast<uint32_t>(audio->opticalRx[offset + 1U]) << 8U) |
	static_cast<uint32_t>(audio->opticalRx[offset + 2U]);
      const uint8_t counter = audio->opticalRx[offset + 3U] >> 4U;
      const uint8_t receivedCrc = audio->opticalRx[offset + 3U] & 0x0FU;
      if (opt4048Crc(exponent, mantissa, counter) != receivedCrc) {
	++audio->opticalCrcErrors;
	crcValid = false;
	break;
      }

      samples[channel] = mantissa << exponent;
      counters[channel] = counter;
    }

    bool countersEqual = crcValid;
    for (size_t channel = 1U; countersEqual && (channel < 4U); ++channel) {
      countersEqual = counters[channel] == counters[0];
    }
    if (crcValid && (not countersEqual)) {
      ++audio->opticalTornFrames;
    }

    if (crcValid && countersEqual) {
      frameValid = true;
      break;
    }

    if ((attempt + 1U) < opticalFrameAttempts) {
      chThdSleepMicroseconds(700U);
    }
  }

  // Invalid CRC/counter alignment is transient and does not mean that the
  // sensor disappeared. Ignore this poll after bounded, phase-shifting retries.
  if (not frameValid) {
    return true;
  }

  const bool hadBackground = audio->opticalBackgroundValid;
  uint32_t positiveAc = 0U;
  float relativeAc[4] = {};
  for (size_t channel = 0U; channel < 4U; ++channel) {
    const uint32_t sample = samples[channel];
    audio->opticalRaw[channel] = sample;
    audio->opticalCounter[channel] = counters[channel];
    if (not hadBackground) {
      audio->opticalBackground[channel] = sample;
      continue;
    }

    const uint32_t background = audio->opticalBackground[channel];
    const int64_t delta = static_cast<int64_t>(sample) -
			  static_cast<int64_t>(background);
    audio->opticalBackground[channel] = static_cast<uint32_t>(
      static_cast<int64_t>(background) + delta / 16);
    if ((delta > 0) && (static_cast<uint64_t>(delta) > positiveAc)) {
      positiveAc = static_cast<uint32_t>(delta);
    }
    if (delta > 0) {
      relativeAc[channel] = static_cast<float>(delta) /
	(static_cast<float>(background) + 1.0f);
    }
  }
  audio->opticalBackgroundValid = true;

  float flashScore = 0.0f;
  if (hadBackground) {
    // OPT4048 channels are CIE X, Y, Z and wideband. A red flash raises X/Y
    // much more than Z; normalizing each delta by its own background rejects
    // common illumination changes such as clouds and vehicle shadows.
    const float redAmplitude = std::max(relativeAc[0], relativeAc[1]);
    const float redExcess = std::max(
      0.0f, 0.5f * (relativeAc[0] + relativeAc[1]) - relativeAc[2]);
    const float colorConfidence = std::clamp(
      redExcess / (redAmplitude + 0.000001f), 0.0f, 1.0f);
    const float redActivity = redAmplitude *
      (0.25f + 0.75f * colorConfidence);

    if (not audio->opticalNoiseFloorValid) {
      audio->opticalNoiseFloor = std::min(redActivity, 0.001f);
      audio->opticalNoiseFloorValid = true;
    } else if (not audio->opticalPulseActive) {
      const float alpha = redActivity < audio->opticalNoiseFloor
	? 1.0f / 8.0f : 1.0f / 128.0f;
      audio->opticalNoiseFloor +=
	(redActivity - audio->opticalNoiseFloor) * alpha;
    }

    const float absoluteScore = knee(redActivity, 0.002f, 0.08f);
    const float riseScore = knee(
      redActivity / (audio->opticalNoiseFloor + 0.00001f), 2.0f, 8.0f);
    const float instantScore = absoluteScore * riseScore *
      (0.25f + 0.75f * colorConfidence);
    if (audio->opticalPulseActive) {
      if (instantScore <= 0.25f) {
	audio->opticalPulseActive = false;
      }
    } else if (instantScore >= 0.65f) {
      audio->opticalPulseActive = true;
      ++audio->opticalPulseCount;
    }

    audio->opticalFlashHold = std::max(
      instantScore, audio->opticalFlashHold * 0.85f);
    flashScore = audio->opticalFlashHold;
  } else {
    audio->opticalFlashHold = 0.0f;
  }

  ++audio->opticalValidFrames;
  const systime_t sampleTime = chVTGetSystemTimeX();
  chSysLock();
  audio->opticalPositiveAc = positiveAc;
  for (size_t channel = 0U; channel < 4U; ++channel) {
    audio->opticalRelativeAc[channel] = relativeAc[channel];
  }
  audio->opticalFlashScore = flashScore;
  audio->opticalLastValidSample = sampleTime;
  chSysUnlock();

  uint16_t status = 0U;
  if (not readOpticalRegister(opt4048RegStatus, status)) {
    return false;
  }
  chSysLock();
  audio->opticalStatus = status;
  chSysUnlock();
  return true;
}

/** @brief Own ADC1, process DMA halves and periodically refresh health. */
void ImavRole::audioThread(void *)
{
  uint32_t lastSequence = 0U;
  bool discardNextBlock = true;
  bool discontinuity = true;
  systime_t lastHealth = chVTGetSystemTimeX();
  systime_t lastBlock = lastHealth;

  while (true) {
    const eventmask_t events =
      chEvtWaitAnyTimeout(allAudioEvents, TIME_MS2I(100U));
    const bool adcError = (events & audioErrorEvent) != 0U;

    uint32_t adcSequence;
    chSysLock();
    adcSequence = audio->adcSequence;
    chSysUnlock();

    if ((not adcError) && (events != 0U) &&
	(adcSequence > lastSequence)) {
      if (adcSequence > (lastSequence + 1U)) {
        audio->droppedBlocks += adcSequence - lastSequence - 1U;
        discontinuity = true;
      }

      const size_t half = (adcSequence - 1U) & 1U;
      if (discardNextBlock) {
        discardNextBlock = false;
      } else {
        processAudioHalf(half * audioHalfDepth, discontinuity);
        discontinuity = false;
      }
      lastSequence = adcSequence;
      lastBlock = chVTGetSystemTimeX();
    }

    const systime_t now = chVTGetSystemTimeX();
    const bool acquisitionStalled =
      chTimeDiffX(lastBlock, now) >= TIME_MS2I(100U);
    if (adcError || (events == 0U) || acquisitionStalled) {
      adcerror_t errors;
      chSysLock();
      errors = audio->errors;
      chSysUnlock();

      stopAudioAcquisition();
      ++audio->droppedBlocks;
      ++audio->acquisitionRestarts;
      if (not adcError) {
	++audio->stalledBlocks;
      }
      if ((audio->acquisitionRestarts <= 4U) ||
	  ((audio->acquisitionRestarts % 32U) == 0U)) {
	m_node->infoCb("IMAV audio restart=%lu stall=%lu adcerr=0x%lx",
		       audio->acquisitionRestarts, audio->stalledBlocks,
		       static_cast<unsigned long>(errors));
      }
      chThdSleepMilliseconds(10U);
      (void) chEvtGetAndClearEvents(allAudioEvents);
      startAudioAcquisition();
      lastSequence = 0U;
      discardNextBlock = true;
      discontinuity = true;
      lastBlock = chVTGetSystemTimeX();
      continue;
    }

    if (chTimeDiffX(lastHealth, now) >= healthPeriod) {
      stopAudioAcquisition();
      const bool healthOk = Adc::sampleOnce();
      (void) chEvtGetAndClearEvents(allAudioEvents);
      startAudioAcquisition();
      lastSequence = 0U;
      discardNextBlock = true;
      discontinuity = true;
      lastHealth = now;
      lastBlock = chVTGetSystemTimeX();
      if (not healthOk) {
        m_node->infoCb("IMAV: ADC health sampling failed");
      }
    }
  }
}

/** @brief Poll the OPT4048 independently so I2C latency cannot stall audio DSP. */
void ImavRole::opticalThread(void *)
{
  systime_t lastRetry = chVTGetSystemTimeX();

  while (true) {
    if (audio->opticalAvailable) {
      if (not readOpticalSensor()) {
	if (++audio->opticalConsecutiveTransportErrors >= 3U) {
	  audio->opticalAvailable = false;
	  audio->opticalConsecutiveTransportErrors = 0U;
	  lastRetry = chVTGetSystemTimeX();
	}
      } else {
	audio->opticalConsecutiveTransportErrors = 0U;
      }
    } else {
      const systime_t now = chVTGetSystemTimeX();
      if (chTimeDiffX(lastRetry, now) >= opticalRetryPeriod) {
	audio->opticalAvailable = initializeOpticalSensor();
	lastRetry = now;
	if (audio->opticalAvailable) {
	  m_node->infoCb("IMAV light recovered: id=0x%04x",
			 audio->opticalDeviceId);
	}
      }
    }

    chThdSleepMilliseconds(10U);
  }
}

#endif // USE_IMAV_ROLE
