/**
 * @file imavRole.cpp
 * @brief IMAV beacon sound and light detection role.
 */

#include "roleConf.h"

#if USE_IMAV_ROLE

#include "imavRole.hpp"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"
#include "imavLightRange.hpp"
#include "resourceManager.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <new>
#include <variant>

namespace {
  constexpr size_t audioBufferDepth = 1024U;
  constexpr size_t audioHalfDepth = audioBufferDepth / 2U;
  constexpr gptcnt_t audioTimerInterval = 1771U;
  constexpr uint16_t audioBandHighHz = 3000U;
  constexpr uint16_t audioBandGuardHz = 50U;
  constexpr uint8_t audioBurstEndLowBlocks = 1U;
  constexpr uint16_t minimumAudioOnsetIntervalMs = 250U;
  constexpr uint16_t maximumAudioOnsetIntervalMs = 750U;
  constexpr uint16_t audioEventTimeoutMs =
    2U * maximumAudioOnsetIntervalMs;
  constexpr float audioOnsetMinimumBlockScore = 0.45f;
  constexpr float audioOnsetMinimumPairScore = 1.20f;
  constexpr float audioOnsetMaximumFrequencyStepHz = 250.0f;
  constexpr sysinterval_t measurementPublishPeriod = TIME_MS2I(200U);
  constexpr sysinterval_t continuousAudioPublishPeriod = TIME_MS2I(200U);
  constexpr sysinterval_t inactiveScorePublishPeriod = TIME_MS2I(1000U);

  // Derive the configurable range from the persistent parameter metadata so
  // its validation and the DSP configuration cannot silently diverge.
  constexpr auto audioBandLowParam =
    Persistant::Parameter::cfind("role.imav.audio.band_low_hz");
  constexpr uint16_t minimumAudioBandLowHz = static_cast<uint16_t>(
    std::get<Persistant::Integer>(audioBandLowParam.second.min));
  constexpr uint16_t maximumAudioBandLowHz = static_cast<uint16_t>(
    std::get<Persistant::Integer>(audioBandLowParam.second.max));
  constexpr uint16_t defaultAudioBandLowHz = static_cast<uint16_t>(
    std::get<Persistant::Integer>(audioBandLowParam.second.v));

  struct GoertzelBin {
    float coefficient;
    uint16_t frequencyHz;
  };

  // Fs = 42.5 MHz / 1771 = 23997.741389 Hz. A fixed 50 Hz grid avoids
  // recomputing coefficients in the acquisition path. The selected alarm bins
  // depend on the persistent lower edge and retain a 50 Hz guard on each side.
  // Two lower reference bins follow that edge; two upper references are fixed.
  constexpr std::array<GoertzelBin, 31U> goertzelBins = {{
    {1.816252304f, 1650U},
    {1.805134501f, 1700U},
    {1.793707339f, 1750U},
    {1.781972776f, 1800U},
    {1.769932824f, 1850U},
    {1.757589546f, 1900U},
    {1.744945058f, 1950U},
    {1.732001526f, 2000U},
    {1.718761168f, 2050U},
    {1.705226254f, 2100U},
    {1.691399104f, 2150U},
    {1.677282086f, 2200U},
    {1.662877621f, 2250U},
    {1.648188176f, 2300U},
    {1.633216270f, 2350U},
    {1.617964468f, 2400U},
    {1.602435383f, 2450U},
    {1.586631678f, 2500U},
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

  struct AudioBandConfiguration {
    uint16_t lowFrequencyHz;
    size_t firstAlarmBin;
    size_t lastAlarmBin;
    size_t alarmBinCount;
    std::array<size_t, 4U> referenceBins;
  };

  constexpr size_t firstBinAtOrAbove(uint16_t frequencyHz)
  {
    for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
      if (goertzelBins[bin].frequencyHz >= frequencyHz) {
	return bin;
      }
    }
    return goertzelBins.size() - 1U;
  }

  constexpr size_t lastBinAtOrBelow(uint16_t frequencyHz)
  {
    for (size_t bin = goertzelBins.size(); bin > 0U; --bin) {
      if (goertzelBins[bin - 1U].frequencyHz <= frequencyHz) {
	return bin - 1U;
      }
    }
    return 0U;
  }

  constexpr size_t closestBin(uint16_t frequencyHz)
  {
    size_t closest = 0U;
    uint16_t closestDistance = UINT16_MAX;
    for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
      const uint16_t binFrequency = goertzelBins[bin].frequencyHz;
      const uint16_t distance = binFrequency >= frequencyHz
	? static_cast<uint16_t>(binFrequency - frequencyHz)
	: static_cast<uint16_t>(frequencyHz - binFrequency);
      if (distance < closestDistance) {
	closest = bin;
	closestDistance = distance;
      }
    }
    return closest;
  }

  constexpr AudioBandConfiguration makeAudioBandConfiguration(
    uint16_t lowFrequencyHz)
  {
    const size_t firstAlarmBin = firstBinAtOrAbove(
      static_cast<uint16_t>(lowFrequencyHz - audioBandGuardHz));
    const size_t lastAlarmBin = lastBinAtOrBelow(
      static_cast<uint16_t>(audioBandHighHz + audioBandGuardHz));
    return {
      .lowFrequencyHz = lowFrequencyHz,
      .firstAlarmBin = firstAlarmBin,
      .lastAlarmBin = lastAlarmBin,
      .alarmBinCount = lastAlarmBin - firstAlarmBin + 1U,
      .referenceBins = {
	closestBin(static_cast<uint16_t>(lowFrequencyHz - 350U)),
	closestBin(static_cast<uint16_t>(lowFrequencyHz - 250U)),
	closestBin(static_cast<uint16_t>(audioBandHighHz + 250U)),
	closestBin(static_cast<uint16_t>(audioBandHighHz + 400U)),
      },
    };
  }

  constexpr auto defaultAudioBand =
    makeAudioBandConfiguration(defaultAudioBandLowHz);
  constexpr auto narrowestAudioBand =
    makeAudioBandConfiguration(maximumAudioBandLowHz);
  static_assert(defaultAudioBandLowHz == minimumAudioBandLowHz);
  static_assert(goertzelBins[defaultAudioBand.firstAlarmBin].frequencyHz == 1950U);
  static_assert(goertzelBins[defaultAudioBand.referenceBins[0]].frequencyHz == 1650U);
  static_assert(goertzelBins[defaultAudioBand.referenceBins[1]].frequencyHz == 1750U);
  static_assert(goertzelBins[narrowestAudioBand.firstAlarmBin].frequencyHz == 2550U);
  static_assert(goertzelBins[narrowestAudioBand.referenceBins[0]].frequencyHz == 2250U);
  static_assert(goertzelBins[narrowestAudioBand.referenceBins[1]].frequencyHz == 2350U);

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
  float signalToNoiseDb = 0.0f;
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
  AudioBandConfiguration band = defaultAudioBand;
  AudioChannelScore channel;
  float dominantFrequencyHz = 0.0f;
  float blockScore = 0.0f;
  float burstPeakScore = 0.0f;
  float recentBurstStrength = 0.0f;
  float cadenceHz = 0.0f;
  float cadenceScore = 0.0f;
  float audioScore = 0.0f;
  float burstPeakSnrDb = 0.0f;
  float recentBurstSnrDb = 0.0f;
  float previousBlockScore = 0.0f;
  float previousDominantFrequencyHz = 0.0f;
  float previousSignalToNoiseDb = 0.0f;
  systime_t onsets[6] = {};
  systime_t lastBlockTime = 0U;
  systime_t lastOnsetTime = 0U;
  systime_t lastToneTime = 0U;
  systime_t snrWindowStartTime = 0U;
  uint8_t onsetCount = 0U;
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
  systime_t lastMeasurementPublishTime = 0U;
  systime_t lastSnrEventTime = 0U;
  systime_t lastSnrZeroPublishTime = 0U;
  uint16_t mean = 0U;
  uint16_t meanAbsoluteDeviation = 0U;
  AudioDetector detector;
  ImavLightRange *lightRange = nullptr;
  bool timeOfFlightEnabled = false;
  bool snrActive = false;
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
    const AudioBandConfiguration& band = detector.band;
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
    float bandPowerSum = 0.0f;
    float peakPower = 0.0f;
    size_t peakBin = band.firstAlarmBin;

    for (size_t bin = 0U; bin < goertzelBins.size(); ++bin) {
      const float q1 = detector.q1[bin];
      const float q2 = detector.q2[bin];
      const float rawPower = q1 * q1 + q2 * q2 -
	goertzelBins[bin].coefficient * q1 * q2;
      const float power = std::max(rawPower, 0.0f);
      detector.power[bin] = power;
      if ((bin >= band.firstAlarmBin) && (bin <= band.lastAlarmBin)) {
	bandPowerSum += power;
	if (power > peakPower) {
	  peakPower = power;
	  peakBin = bin;
	}
      }
    }

    float referencePower = 0.0f;
    for (const size_t bin : band.referenceBins) {
      referencePower += detector.power[bin];
    }
    referencePower /= static_cast<float>(band.referenceBins.size());
    const float bandPower =
      bandPowerSum / static_cast<float>(band.alarmBinCount);

    score.bandPower = bandPower;
    score.peakPower = peakPower;
    score.referencePower = referencePower;
    // Sum the complete alarm band so a chirp remains concentrated even while
    // its instantaneous peak moves between bins. Factor 2 compensates the
    // Hann window overlap; a band-limited signal then approaches 0 dB.
    score.concentration = std::clamp(
      2.0f * bandPowerSum /
	(static_cast<float>(audioHalfDepth) * windowedEnergy + 1.0f),
      0.0f, 1.0f);
    score.spectralRatioDb = 10.0f * std::log10(
      std::max(score.concentration, minimumSpectralRatio));
    score.prominence = bandPower / (referencePower + 1.0f);
    // Hann coherent gain gives A_rms ~= sqrt(Goertzel power) * sqrt(8) / N.
    score.toneRms = std::sqrt(peakPower) * 0.005524272f;
    const float prominenceScore = knee(score.prominence, 1.5f, 5.0f);
    const float bandEnergyScore = knee(
      score.spectralRatioDb, -15.0f, -6.0f);
    // A rise above the learned floor alone also accepts broadband impulses.
    // Require the alarm band to stand above the side-band references; the
    // concentration term still accepts a chirp moving inside the full band.
    score.blockScore = bandEnergyScore * prominenceScore;
    if (clipped >= 5U) {
      score.blockScore *= 0.8f;
    }

    // Learn only blocks that do not already resemble the beacon. This robust
    // floor follows continuous broadband motor noise but cannot slowly absorb
    // a persistent alarm into its own reference level. If acquisition starts
    // inside the alarm, seed it from the side bands instead of treating the
    // already-present signal as background. Both powers are averages per bin.
    if (not score.floorValid) {
      score.noiseFloor = score.blockScore > 0.30f ? referencePower : bandPower;
      score.floorValid = true;
    } else if ((detector.burstState != AudioBurstState::On) &&
	       (score.blockScore <= 0.30f)) {
      const float alpha = bandPower < score.noiseFloor ? 1.0f / 16.0f
						    : 1.0f / 256.0f;
      score.noiseFloor += (bandPower - score.noiseFloor) * alpha;
    }
    const float signalPower = std::max(bandPower - score.noiseFloor, 0.0f);
    score.signalToNoiseDb = std::clamp(10.0f * std::log10(
      (signalPower + 1.0f) / (score.noiseFloor + 1.0f)), -60.0f, 60.0f);
    score.peakFrequencyHz = goertzelBins[peakBin].frequencyHz;
    score.minimum = minimum;
    score.maximum = maximum;
    score.clippedSamples = clipped;

    float strongestPower = 0.0f;
    size_t strongestBin = band.firstAlarmBin;
    for (size_t bin = band.firstAlarmBin; bin <= band.lastAlarmBin; ++bin) {
      if (detector.power[bin] > strongestPower) {
	strongestPower = detector.power[bin];
	strongestBin = bin;
      }
    }

    float binOffset = 0.0f;
    if ((strongestBin > band.firstAlarmBin) &&
	(strongestBin < band.lastAlarmBin)) {
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

  /** @brief Drop interrupted evidence while retaining the learned noise floor. */
  void clearAudioCadence(AudioDetector& detector)
  {
    detector.onsetCount = 0U;
    detector.lastOnsetTime = 0U;
    detector.lastToneTime = 0U;
    detector.snrWindowStartTime = 0U;
    detector.cadenceHz = 0.0f;
    detector.cadenceScore = 0.0f;
    detector.audioScore = 0.0f;
    detector.recentBurstStrength = 0.0f;
    detector.burstPeakScore = 0.0f;
    detector.burstPeakSnrDb = 0.0f;
    detector.recentBurstSnrDb = 0.0f;
    detector.previousBlockScore = 0.0f;
    detector.previousDominantFrequencyHz = 0.0f;
    detector.previousSignalToNoiseDb = 0.0f;
    detector.lowBlocks = 0U;
    detector.burstState = AudioBurstState::Unarmed;
    detector.detected = false;
  }

  /** @brief Append an onset to the fixed six-entry chronological history. */
  void appendAudioOnset(AudioDetector& detector, systime_t now)
  {
    if (detector.lastOnsetTime != 0U) {
      const sysinterval_t onsetInterval =
	chTimeDiffX(detector.lastOnsetTime, now);
      if (onsetInterval < TIME_MS2I(minimumAudioOnsetIntervalMs)) {
	return;
      }
      if (onsetInterval > TIME_MS2I(maximumAudioOnsetIntervalMs)) {
	// Do not mix a new burst train with stale cadence evidence.
	detector.onsetCount = 0U;
      }
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

  /** @brief Finalize the SNR peak of a burst or a continuous-sound window. */
  void finalizeAudioSnr(AudioDetector& detector)
  {
    if (detector.recentBurstSnrDb <= 0.0f) {
      detector.recentBurstSnrDb = detector.burstPeakSnrDb;
    } else {
      const float alpha = std::clamp(
        param_cget<"role.imav.audio.snr_alpha">(), 0.5f, 1.0f);
      detector.recentBurstSnrDb += alpha *
        (detector.burstPeakSnrDb - detector.recentBurstSnrDb);
    }
    detector.burstPeakSnrDb = 0.0f;
  }

  /** @brief Measure cadence; return true when a new SNR is ready to publish. */
  bool updateAudioCadence(AudioDetector& detector, bool discontinuity,
                          systime_t now)
  {
    bool snrReady = false;
    if (discontinuity || ((detector.lastBlockTime != 0U) &&
	(chTimeDiffX(detector.lastBlockTime, now) >= TIME_MS2I(200U)))) {
      clearAudioCadence(detector);
    }
    detector.lastBlockTime = now;

    const bool low = detector.blockScore <= 0.30f;
    switch (detector.burstState) {
    case AudioBurstState::Unarmed:
    case AudioBurstState::Off: {
      if (detector.burstState == AudioBurstState::Unarmed) {
        detector.lowBlocks = low
          ? static_cast<uint8_t>(detector.lowBlocks + 1U) : 0U;
        if (detector.lowBlocks >= 2U) {
          detector.burstState = AudioBurstState::Off;
          detector.lowBlocks = 0U;
        }
      } else {
        detector.lowBlocks = 0U;
      }

      // A weak chirp over broadband noise can put one of two adjacent blocks
      // just below the strong threshold. Accept the pair only when its total
      // score is high and its spectral peak moves coherently; random broadband
      // peaks do not retain a nearby dominant frequency from block to block.
      const bool coherentPair =
	detector.previousBlockScore >= audioOnsetMinimumBlockScore &&
	detector.blockScore >= audioOnsetMinimumBlockScore &&
	(detector.previousBlockScore + detector.blockScore) >=
	  audioOnsetMinimumPairScore &&
	std::fabs(detector.dominantFrequencyHz -
		  detector.previousDominantFrequencyHz) <=
	  audioOnsetMaximumFrequencyStepHz;
      if (coherentPair) {
        // A continuous alarm can already be present at startup or after a
        // lost block. Qualify it with two fresh blocks, without requiring a
        // silence first. Only an observed Off -> On transition is a cadence
        // onset; reacquisition must not invent a new beep.
        const bool observedOnset = detector.burstState == AudioBurstState::Off;
	detector.burstState = AudioBurstState::On;
	detector.burstPeakScore = std::max(
	  detector.previousBlockScore, detector.blockScore);
	detector.burstPeakSnrDb = std::max(
	  {0.0f, detector.previousSignalToNoiseDb,
	   detector.channel.signalToNoiseDb});
	detector.lastToneTime = now;
	detector.snrWindowStartTime = now;
        if (observedOnset) {
          appendAudioOnset(detector, now);
        }
      }
      break;
    }

    case AudioBurstState::On:
      detector.lastToneTime = now;
      detector.burstPeakScore =
	std::max(detector.burstPeakScore, detector.blockScore);
      detector.burstPeakSnrDb = std::max(
	detector.burstPeakSnrDb, detector.channel.signalToNoiseDb);
      detector.lowBlocks = low ? static_cast<uint8_t>(detector.lowBlocks + 1U)
			       : 0U;
      // A powerful loudspeaker and the real beacon can leave a narrow-band
      // acoustic tail in most of the nominally silent interval. One complete
      // 21 ms block below the low threshold is nevertheless an unambiguous
      // gap, and is needed to separate 120 ms bursts repeated at 3 Hz.
      if (detector.lowBlocks >= audioBurstEndLowBlocks) {
        detector.burstState = AudioBurstState::Off;
        detector.lowBlocks = 0U;
        detector.recentBurstStrength = detector.burstPeakScore;
        finalizeAudioSnr(detector);
        snrReady = true;
        detector.burstPeakScore = 0.0f;
      } else if (chTimeDiffX(detector.snrWindowStartTime, now) >=
                 continuousAudioPublishPeriod) {
        // Short bursts publish only at their end. A sustained sound instead
        // publishes the peak of each 200 ms window, allowing falling levels
        // to be observed without waiting for silence. Preserve the detection
        // state and cadence; only the SNR accumulator starts a new window.
        finalizeAudioSnr(detector);
        snrReady = true;
        detector.snrWindowStartTime += continuousAudioPublishPeriod;
        if (chTimeDiffX(detector.snrWindowStartTime, now) >=
            continuousAudioPublishPeriod) {
          detector.snrWindowStartTime = now;
        }
      }
      break;
    }

    detector.previousBlockScore = detector.blockScore;
    detector.previousDominantFrequencyHz = detector.dominantFrequencyHz;
    detector.previousSignalToNoiseDb = detector.channel.signalToNoiseDb;

    detector.cadenceHz = 0.0f;
    detector.cadenceScore = 0.0f;
    if (detector.lastOnsetTime != 0U) {
      const uint32_t onsetAgeMs = TIME_I2MS(
	chTimeDiffX(detector.lastOnsetTime, now));
      if (onsetAgeMs >= audioEventTimeoutMs) {
	detector.onsetCount = 0U;
	detector.lastOnsetTime = 0U;
      } else {
	float periodScoreSum = 0.0f;
	float periodMsSum = 0.0f;
	const uint8_t intervalCount = detector.onsetCount > 0U
	  ? static_cast<uint8_t>(detector.onsetCount - 1U) : 0U;
	for (uint8_t index = 1U; index < detector.onsetCount; ++index) {
	  const float periodMs = static_cast<float>(TIME_I2MS(
	    chTimeDiffX(detector.onsets[index - 1U],
			detector.onsets[index])));
	  const float prealarmError = (periodMs - 500.0f) / 90.0f;
	  const float alarmError = (periodMs - 333.333f) / 80.0f;
	  const float prealarmScore =
	    1.0f / (1.0f + prealarmError * prealarmError);
	  const float alarmScore =
	    1.0f / (1.0f + alarmError * alarmError);
	  periodScoreSum += std::max(prealarmScore, alarmScore);
	  periodMsSum += periodMs;
	}
	if (intervalCount != 0U) {
	  detector.cadenceHz = 1000.0f / (periodMsSum / intervalCount);
	  const float support = std::min(
	    static_cast<float>(intervalCount) / 3.0f, 1.0f);
	  detector.cadenceScore =
	    (periodScoreSum / intervalCount) * support;
	}
      }
    }

    // Cadence is diagnostic only. Two consecutive spectral blocks in the
    // configured band validate both prealarm and full alarm, and the evidence
    // is held long enough to bridge their silent intervals.
    const float strength = detector.burstState == AudioBurstState::On
      ? std::max(detector.recentBurstStrength, detector.burstPeakScore)
      : detector.recentBurstStrength;
    float toneFreshness = 0.0f;
    if (detector.lastToneTime != 0U) {
      const uint32_t toneAgeMs = TIME_I2MS(
	chTimeDiffX(detector.lastToneTime, now));
      toneFreshness = toneAgeMs <= 750U ? 1.0f :
	std::max(0.0f,
	  (static_cast<float>(audioEventTimeoutMs) -
	   static_cast<float>(toneAgeMs)) / 750.0f);
    }
    detector.audioScore = strength * toneFreshness;
    if (detector.detected) {
      detector.detected = detector.audioScore >= 0.25f;
    } else {
      detector.detected = detector.audioScore >= 0.60f;
    }
    return snrReady;
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
  if (not boardResource.tryAcquire(
        HR::PA04, HR::PA08, HR::ADC_2, HR::TIM_6)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::ADC_2));
  }

  if (ADCD2.state != ADC_STOP) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::ADC_2));
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
      0U,
      ADC_SMPR2_SMP_AN17(ADC_SMPR_SMP_47P5),
    },
    .sqr = {
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN17),
      0U,
      0U,
      0U,
    },
  };

  audio->adcGroup = &adcAudioGroup;
  audio->timeOfFlightEnabled =
    param_cget<"role.imav.time_of_flight">();
  const systime_t startTime = chVTGetSystemTimeX();
  audio->lastMeasurementPublishTime = startTime;
  audio->lastSnrEventTime = startTime;
  audio->lastSnrZeroPublishTime = startTime;
  audio->detector.band = makeAudioBandConfiguration(static_cast<uint16_t>(
    param_cget<"role.imav.audio.band_low_hz">()));

  // OPT4060 supports standard mode and Fast mode, but not the STM32G4
  // Fast-mode Plus setting at 1 MHz (its next supported mode is HS 2.6 MHz).
  const uint32_t i2cFrequencyKhz = param_cget<"bus.i2c.frequency_khz">();
  if ((i2cFrequencyKhz < 100U) || (i2cFrequencyKhz > 400U)) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE,
			DeviceStatus::I2C_FREQ_INVALID,
			static_cast<uint16_t>(i2cFrequencyKhz));
  }

  const DeviceStatus i2cStatus = I2CPeriph::start();
  if (not i2cStatus) {
    return i2cStatus;
  }

  void * const sensorMemory = malloc_m(sizeof(ImavLightRange));
  if (sensorMemory == nullptr) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE, DeviceStatus::HEAP_FULL);
  }
  audio->lightRange = new (sensorMemory) ImavLightRange(
    node,
    static_cast<uint8_t>(
      param_cget<"role.imav.light.i2c_address">()),
    static_cast<uint32_t>(
      param_cget<"role.imav.tof.period_ms">()),
    audio->timeOfFlightEnabled,
    param_cget<"role.imav.light.beginning_pattern">(),
    static_cast<uint16_t>(param_cget<"role.imav.light.high_ms">()),
    static_cast<uint16_t>(param_cget<"role.imav.light.steady_low_ms">()),
    static_cast<uint16_t>(
      param_cget<"role.imav.light.beginning_low_ms">()),
    param_cget<"role.imav.light.cree_test">(),
    param_cget<"role.imav.light.adaptive_pattern">());
  audio->lightRange->initialize();
  const ImavLightRangeSnapshot initialSensors =
    audio->lightRange->snapshot();

  palSetLineMode(LINE_SPI_PERIPH_CS, PAL_MODE_INPUT_ANALOG);

  // ADCv3 asserts instead of returning an error when its dynamically chosen
  // stream is exhausted. IMAV is started before optional roles; this explicit
  // preflight therefore turns the remaining failure case into DeviceStatus.
  const stm32_dma_stream_t * const dmaProbe = dmaStreamAlloc(
    STM32_ADC_ADC2_DMA_STREAM, STM32_ADC_ADC2_DMA_IRQ_PRIORITY,
    nullptr, nullptr);
  if (dmaProbe == nullptr) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE,
			DeviceStatus::DMA_UNAVAILABLE,
			std::to_underlying(HR::ADC_2));
  }
  dmaStreamFree(dmaProbe);

  if (adcStart(&ADCD2, nullptr) != MSG_OK) {
    return DeviceStatus(DeviceStatus::IMAV_ROLE, DeviceStatus::NOT_RESPONDING,
			std::to_underlying(HR::ADC_2));
  }
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
    nullptr, THD_WORKING_AREA_SIZE(2048U), "imav sensors", NORMALPRIO,
    &Trampoline<&ImavRole::opticalThread>::fn, this);
  if (audio->opticalWorker == nullptr) {
    node.infoCb("IMAV sensor worker unavailable: heap full");
  }

  node.infoCb("IMAV audio started: PA4/ADC2, 24kHz, OVS x4, band=%u-%uHz",
	      audio->detector.band.lowFrequencyHz, audioBandHighHz);
  if (initialSensors.lightAvailable) {
    node.infoCb("IMAV light started: OPT4060 addr=0x%02x id=0x%04x INT=PA8",
			initialSensors.lightAddress,
		static_cast<unsigned>(initialSensors.lightDeviceId));
  } else {
    node.infoCb("IMAV light unavailable: OPT4060 addr=0x44..0x47");
  }
  if (audio->timeOfFlightEnabled) {
    if (initialSensors.rangeAvailable) {
      node.infoCb("IMAV range started: VL53L4CX id=0x%04lx",
		  static_cast<unsigned long>(initialSensors.rangeDeviceId));
    } else {
      node.infoCb("IMAV range unavailable: VL53L4CX addr=0x29");
    }
  } else {
    node.infoCb("IMAV range disabled");
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

  adcStartConversion(&ADCD2, audio->adcGroup, audio->samples,
		     audioBufferDepth);
  gptStartContinuous(&GPTD6, audioTimerInterval);
}

/** @brief Stop the trigger first, then return ADC2 to READY state. */
void ImavRole::stopAudioAcquisition()
{
  gptStopTimer(&GPTD6);
  adcStopConversion(&ADCD2);
}

/** @brief Minimal ISR callback: count half-buffers and wake the worker. */
void ImavRole::audioDmaCallback(ADCDriver *adcp)
{
  chSysLockFromISR();
  if (adcp == &ADCD2) {
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
  const systime_t now = chVTGetSystemTimeX();
  const bool snrReady =
    updateAudioCadence(audio->detector, discontinuity, now);
  ++audio->processedBlocks;
  if (discontinuity) {
    ++audio->discontinuities;
  }

  if (snrReady) {
    publishAudioSnr();
  }

  if (chTimeDiffX(audio->lastMeasurementPublishTime, now) >=
      measurementPublishPeriod) {
    audio->lastMeasurementPublishTime += measurementPublishPeriod;
    if (chTimeDiffX(audio->lastMeasurementPublishTime, now) >=
	measurementPublishPeriod) {
      // Do not emit a catch-up burst if the worker was delayed.
      audio->lastMeasurementPublishTime = now;
    }
    publishMeasurements();
  }

  // IMAV measurements and detailed diagnostics are published over CAN.
  // Keep the serial debug link quiet during normal operation.
}

/** @brief Publish the finalized SNR of a burst or continuous-sound window. */
void ImavRole::publishAudioSnr()
{
  audio->lastSnrEventTime = chVTGetSystemTimeX();
  audio->snrActive = true;
  uavcan_protocol_debug_KeyValue message = {};
  message.value = audio->detector.recentBurstSnrDb;
  UAVCAN::dsdlAssign(message.key, "snr");
  m_node->sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW);
}

/** @brief Publish navigation measurements as compact single-frame values. */
void ImavRole::publishMeasurements()
{
  uavcan_protocol_debug_KeyValue message = {};
  const auto publish = [this, &message](const char *key, float value) {
    message.value = value;
    UAVCAN::dsdlAssign(message.key, key);
    m_node->sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW);
  };
  const systime_t now = chVTGetSystemTimeX();
  const bool snrExpired =
    chTimeDiffX(audio->lastSnrEventTime, now) >=
      TIME_MS2I(audioEventTimeoutMs);
  if (audio->snrActive && snrExpired) {
    audio->snrActive = false;
    audio->detector.recentBurstSnrDb = 0.0f;
    publish("snr", 0.0f);
    audio->lastSnrZeroPublishTime = now;
  } else if ((not audio->snrActive) && snrExpired &&
             (chTimeDiffX(audio->lastSnrZeroPublishTime, now) >=
              inactiveScorePublishPeriod)) {
    publish("snr", 0.0f);
    audio->lastSnrZeroPublishTime = now;
  }

  if (not param_cget<"role.imav.debug.publish.optional">()) {
    return;
  }

  const ImavLightRangeSnapshot sensors = audio->lightRange != nullptr
    ? audio->lightRange->snapshot() : ImavLightRangeSnapshot{};
  publish("a0", audio->detector.channel.blockScore);
  publish("p0", audio->detector.channel.toneRms);
  publish("sdb", audio->detector.channel.spectralRatioDb);
  publish("aud", audio->detector.audioScore);
  publish("frq", audio->detector.dominantFrequencyHz);
  publish("cad", audio->detector.cadenceHz);
  publish("lrr", sensors.lightRedRatio);
  publish("lac", sensors.lightRelativeAc);
  publish("lis", sensors.lightInstantScore);
  publish("lhz", sensors.lightCadenceHz);
  publish("lcs", sensors.lightCadenceScore);
  publish("lfs", sensors.lightFastScore);
  publish("lps", sensors.lightPulseStrength);
  publish("lpc", static_cast<float>(sensors.lightPulses));
  publish("lsa", static_cast<float>(sensors.lightSaturations));
  publish("ler", static_cast<float>(sensors.lightReadErrors));
  publish("lgp", static_cast<float>(sensors.lightGaps));
  publish("lon", sensors.lightHighDurationMs);
  publish("lof", sensors.lightLowDurationMs);
  publish("lts", sensors.lightTemporalShapeScore);
  // 0=none, 1=startup, 2=steady, 3=CREE bench, 4=learned repeating pattern.
  publish("lpt", static_cast<float>(sensors.lightPattern));
  publish("lpn", static_cast<float>(sensors.lightPatternPulses));
  if (audio->timeOfFlightEnabled) {
    const float rangeMetres = sensors.rangeValid
      ? static_cast<float>(sensors.rangeMm) * 0.001f : -1.0f;
    publish("rng", rangeMetres);
    publish("rsg", sensors.rangeSignalKcps);
  }
}

/** @brief Own ADC2 and process each completed DMA half-buffer. */
void ImavRole::audioThread(void *)
{
  uint32_t lastSequence = 0U;
  bool discardNextBlock = true;
  bool discontinuity = true;
  systime_t lastBlock = chVTGetSystemTimeX();

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
  }
}

/** @brief Sample RGBW light and schedule exclusive ToF measurements. */
void ImavRole::opticalThread(void *)
{
  if (audio->lightRange != nullptr) {
    audio->lightRange->run();
  }
  while (true) {
    chThdSleepMilliseconds(1000U);
  }
}

#endif // USE_IMAV_ROLE
