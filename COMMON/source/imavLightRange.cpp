/**
 * @file imavLightRange.cpp
 * @brief OPT4060 RGBW flash detection and VL53L4CX ground ranging.
 */

#include "roleConf.h"

#if USE_IMAV_ROLE

#include "imavLightRange.hpp"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"

#include <uavcan.protocol.debug.KeyValue.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
  constexpr uint8_t opt4060FirstAddress = 0x44U;
  constexpr uint8_t opt4060LastAddress = 0x47U;
  constexpr uint16_t opt4060ExpectedId = 0x0821U;

  constexpr uint8_t optRegChannel0Msb = 0x00U;
  constexpr uint8_t optRegConfiguration = 0x0AU;
  constexpr uint8_t optRegConfiguration2 = 0x0BU;
  constexpr uint8_t optRegStatus = 0x0CU;
  constexpr uint8_t optRegDeviceId = 0x11U;

  constexpr uint16_t optRangeAuto = 0x0CU << 10U;
  constexpr uint16_t optConversionTime1p8Ms = 2U << 6U;
  constexpr uint16_t optOperatingContinuous = 3U << 4U;
  constexpr uint16_t optInterruptLatch = 1U << 3U;
  constexpr uint16_t optConfigurationPowerDown =
    optRangeAuto | optConversionTime1p8Ms | optInterruptLatch;
  constexpr uint16_t optConfigurationContinuous =
    optConfigurationPowerDown | optOperatingContinuous;
  // Keep required bits at their reset value, INT as an open-drain output and
  // burst reads enabled. INT_CFG=3 emits one active-low 1 us pulse after all
  // four RGBW channels have completed a conversion.
  constexpr uint16_t optInterruptDataReadyAllChannels = 3U << 2U;
  constexpr uint16_t optConfiguration2 =
    0x8011U | optInterruptDataReadyAllChannels;
  constexpr uint16_t optStatusOverload = 1U << 3U;
  constexpr size_t optChannelCount = 4U;
  constexpr size_t optResultBytes = optChannelCount * 4U;
  constexpr uint32_t optPollPeriodMs = 10U;
  constexpr uint32_t optFirstConversionDelayMs = 8U;
  constexpr uint32_t optDataReadyTimeoutMs = 25U;

  // Streaming DFT bins in tenths of hertz. The dense 2.2..3.8 Hz bank makes
  // the noise estimate local and rejects broad outdoor flicker or spectral
  // slopes. Bins 2.8..3.2 Hz search the alarm; 6.0 Hz observes its expected
  // second harmonic without making that weaker component mandatory.
  constexpr std::array<uint8_t, 18U> lightSpectralTenthsHz = {
    22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U,
    31U, 32U, 33U, 34U, 35U, 36U, 37U, 38U, 60U,
  };
  constexpr size_t lightFirstTargetBin = 6U;
  constexpr size_t lightTargetBinCount = 5U;
  constexpr size_t lightHarmonicBin = 17U;
  constexpr uint32_t lightSpectralUpdateDivider = 20U;
  constexpr uint32_t lightSpectralResetGapMs = 100U;
  constexpr float lightFastTrackingEnterScore = 0.55f;
  constexpr float lightFastTrackingExitScore = 0.15f;
  constexpr float lightSlowTrackingExitScore = 0.20f;
  constexpr float twoPi = 6.2831853071795864769f;

  static_assert(optConfigurationPowerDown == 0x3088U);
  static_assert(optConfigurationContinuous == 0x30B8U);
  static_assert(optConfiguration2 == 0x801DU);

  struct Opt4060ChannelSample {
    uint32_t adcCode;
    uint8_t counter;
    bool valid;
  };

  constexpr uint16_t readBigEndian16(const uint8_t *bytes)
  {
    return static_cast<uint16_t>(
      static_cast<uint16_t>(bytes[0]) << 8U | bytes[1]);
  }

  constexpr Opt4060ChannelSample decodeOpt4060Channel(uint16_t msb,
                                                       uint16_t lsb)
  {
    const uint8_t exponent = static_cast<uint8_t>(msb >> 12U);
    const uint32_t mantissa =
      (static_cast<uint32_t>(msb & 0x0FFFU) << 8U) |
      static_cast<uint32_t>(lsb >> 8U);
    return {
      .adcCode = exponent <= 6U ? mantissa << exponent : 0U,
      .counter = static_cast<uint8_t>((lsb >> 4U) & 0x0FU),
      .valid = exponent <= 6U,
    };
  }

  static_assert(
    decodeOpt4060Channel(0x3123U, 0x45A0U).adcCode ==
      (0x12345U << 3U));
  static_assert(decodeOpt4060Channel(0x3123U, 0x45A0U).counter == 0x0AU);

  constexpr uint32_t sensorRetryPeriodMs = 5000U;
  constexpr uint32_t rangeTimingBudgetMs = 30U;
  constexpr float rangeFieldOfViewRad = 0.3141592654f;
  constexpr float rangeMaximumMetres = 6.0f;

  float knee(float value, float low, float high)
  {
    return std::clamp((value - low) / (high - low), 0.0f, 1.0f);
  }
}

ImavLightRange::ImavLightRange(UAVCAN::Node& node_, uint8_t address,
                               uint32_t periodMs, bool enableTimeOfFlight)
  : node(node_),
    lightAddress(address),
    rangePeriodMs(std::clamp(
      periodMs, uint32_t{100U}, uint32_t{1000U})),
    timeOfFlightEnabled(enableTimeOfFlight)
{
  chDbgAssert(active == nullptr, "single IMAV light/range instance expected");
  active = this;
  published.lightAddress = address;
}

extern "C" uint8_t *imav_vl53l4cx_work_buffer(const void *device,
                                               uint32_t requiredSize)
{
  ImavLightRange *const sensors = ImavLightRange::active;
  if ((sensors == nullptr) || (device != &sensors->rangeDevice) ||
      (requiredSize > sizeof(sensors->tofTx))) {
    return nullptr;
  }
  return sensors->tofTx;
}

void ImavLightRange::initialize()
{
  // INT is an active-low open-drain output. PAL's synchronous wait API lets
  // the sensor thread sleep directly on EXTI8 without an application ISR or
  // a periodic polling loop.
  palSetLineMode(LINE_OPT4060_INT, PAL_MODE_INPUT_PULLUP);
  palEnableLineEvent(LINE_OPT4060_INT, PAL_EVENT_MODE_FALLING_EDGE);

  // Ref-SPAD initialization can emit 940 nm light. Configure it before the
  // OPT4060 starts, so the two sensors are never acquiring simultaneously.
  rangeAvailable = timeOfFlightEnabled && initializeRange();
  const bool lightOk = initializeLight();
  (void) lightOk;
  publishAvailability();
}

ImavLightRangeSnapshot ImavLightRange::snapshot() const
{
  chSysLock();
  const ImavLightRangeSnapshot copy = published;
  chSysUnlock();
  return copy;
}

void ImavLightRange::publishAvailability()
{
  chSysLock();
  published.lightAvailable = lightAvailable;
  published.rangeAvailable = rangeAvailable;
  published.rangeDeviceId = rangeDeviceId;
  published.lightAddress = lightAddress;
  published.lightReadErrors = lightReadErrors;
  published.lightBusRecoveries = lightBusRecoveries;
  published.lightGaps = lightGaps;
  published.rangeErrors = rangeErrors;
  chSysUnlock();
}

void ImavLightRange::publishLightState()
{
  chSysLock();
  published.lightRed = lightRed;
  published.lightGreen = lightGreen;
  published.lightBlue = lightBlue;
  published.lightWide = lightWide;
  published.lightRedRatio = lightRedRatio;
  published.lightRelativeAc = lightRelativeAc;
  published.lightInstantScore = lightInstantScore;
  published.lightCadenceHz = lightCadenceHz;
  published.lightCadenceScore = lightCadenceScore;
  published.lightFastScore = lightFastScore;
  published.lightPulseStrength = lightPulseActive
    ? std::max(lightRecentPulseStrength, lightPulsePeakScore)
    : lightRecentPulseStrength;
  published.lightFlashScore = lightFlashScore;
  published.lightSpectralScore = lightSpectralScore;
  published.lightSpectralSnrDb = lightSpectralSnrDb;
  published.lightSpectralCoherence = lightSpectralCoherence;
  published.lightSpectralRedFraction = lightSpectralRedFraction;
  published.lightSpectralFrequencyHz = lightSpectralFrequencyHz;
  published.lightHarmonicRatio = lightHarmonicRatio;
  published.lightSamples = lightSamples;
  published.lightPulses = lightPulses;
  published.lightSaturations = lightSaturations;
  published.lightReadErrors = lightReadErrors;
  published.lightBusRecoveries = lightBusRecoveries;
  published.lightGaps = lightGaps;
  published.lightLastSample = lightLastSample;
  chSysUnlock();
}

void ImavLightRange::publishLightDebugSample(bool overloaded)
{
  if (not param_cget<"role.imav.debug.publish.optional">()) {
    return;
  }

  uavcan_protocol_debug_KeyValue message = {};
  const auto publish = [this, &message](const char *key, float value) {
    message.value = value;
    UAVCAN::dsdlAssign(message.key, key);
    node.sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW);
  };

  // ADC codes are exactly representable as float: the OPT4060 result has a
  // 20-bit mantissa shifted by its exponent, hence at most 20 significant
  // bits even though the linearized code spans 26 bits.
  publish("lrd", static_cast<float>(lightRed));
  publish("lgn", static_cast<float>(lightGreen));
  publish("lbl", static_cast<float>(lightBlue));
  publish("lwh", static_cast<float>(lightWide));
  publish("lov", overloaded ? 1.0f : 0.0f);
  publish("lct", static_cast<float>(lightSamples));
  // Microseconds modulo 2^24 remain exactly representable as float and can be
  // unwrapped by the recorder every 16.78 seconds.
  publish("ltu", static_cast<float>(
    TIME_I2US(lightLastSample) & 0x00FFFFFFU));
}

void ImavLightRange::recordI2cFailure(bool rangeSensor, msg_t result)
{
  if (rangeSensor) {
    ++rangeErrors;
  } else {
    ++lightReadErrors;
  }

  const i2cflags_t errors = i2cGetErrors(&ExternalI2CD);
  if ((result == MSG_TIMEOUT) ||
      ((errors & ~static_cast<i2cflags_t>(I2C_ACK_FAILURE)) !=
       I2C_NO_ERROR)) {
    ++lightBusRecoveries;
    I2CPeriph::resetLocked();
  }
}

msg_t ImavLightRange::lightTransfer(size_t txLength, size_t rxLength)
{
  i2cAcquireBus(&ExternalI2CD);
  const msg_t result = i2cMasterTransmitTimeout(
    &ExternalI2CD, lightAddress, lightTx, txLength, lightRx, rxLength,
    TIME_MS2I(20U));
  if (result != MSG_OK) {
    recordI2cFailure(false, result);
  }
  i2cReleaseBus(&ExternalI2CD);
  return result;
}

bool ImavLightRange::readLightRegister(uint8_t reg, uint16_t& value)
{
  lightTx[0] = reg;
  if (lightTransfer(1U, 2U) != MSG_OK) {
    return false;
  }
  value = readBigEndian16(lightRx);
  return true;
}

bool ImavLightRange::readLightBlock(uint8_t reg, size_t length)
{
  if (length > sizeof(lightRx)) {
    ++lightReadErrors;
    return false;
  }
  lightTx[0] = reg;
  return lightTransfer(1U, length) == MSG_OK;
}

bool ImavLightRange::writeLightRegister(uint8_t reg, uint16_t value)
{
  lightTx[0] = reg;
  lightTx[1] = static_cast<uint8_t>(value >> 8U);
  lightTx[2] = static_cast<uint8_t>(value);
  return lightTransfer(3U, 0U) == MSG_OK;
}

bool ImavLightRange::initializeLight()
{
  lightAvailable = false;
  const uint8_t preferred =
    ((lightAddress >= opt4060FirstAddress) &&
     (lightAddress <= opt4060LastAddress))
    ? lightAddress : opt4060FirstAddress;

  uint16_t id = 0U;
  bool found = false;
  for (uint8_t offset = 0U;
       offset <= (opt4060LastAddress - opt4060FirstAddress); ++offset) {
    const uint8_t candidate = static_cast<uint8_t>(
      opt4060FirstAddress +
      ((preferred - opt4060FirstAddress + offset) % optChannelCount));
    lightAddress = candidate;
    if (readLightRegister(optRegDeviceId, id) &&
        ((id & 0x3FFFU) == opt4060ExpectedId)) {
      found = true;
      break;
    }
  }
  if (not found) {
    publishAvailability();
    return false;
  }

  // A successful power-down write is the invariant which gates every ToF
  // measurement. The OPT4060 still responds to I2C in this mode.
  if (not writeLightRegister(optRegConfiguration,
                             optConfigurationPowerDown)) {
    publishAvailability();
    return false;
  }
  lightRunning = false;
  if (not writeLightRegister(optRegConfiguration2, optConfiguration2)) {
    publishAvailability();
    return false;
  }

  lightBaselineValid = false;
  lightNoiseValid = false;
  lightCountersValid = false;
  lightOverloadActive = false;
  lightFastTracking = false;
  lightConsecutiveErrors = 0U;
  lightStalePolls = 0U;
  lightRedRatio = 0.0f;
  lightRelativeAc = 0.0f;
  lightInstantScore = 0.0f;
  lightLastSample = 0U;
  clearLightCadence();
  resetLightSpectrum();
  if (not startLight()) {
    publishAvailability();
    return false;
  }
  chThdSleepMilliseconds(optFirstConversionDelayMs);

  lightAvailable = true;
  chSysLock();
  published.lightAddress = lightAddress;
  published.lightDeviceId = id;
  published.lightAvailable = true;
  published.lightLastSample = 0U;
  chSysUnlock();
  return true;
}

bool ImavLightRange::startLight()
{
  if (not writeLightRegister(optRegConfiguration,
                             optConfigurationContinuous)) {
    return false;
  }
  lightStalePolls = 0U;
  lightRunning = true;
  return true;
}

bool ImavLightRange::stopLight()
{
  if (not writeLightRegister(optRegConfiguration,
                             optConfigurationPowerDown)) {
    return false;
  }
  lightRunning = false;
  return true;
}

void ImavLightRange::clearLightCadence()
{
  lightOnsets.fill(0U);
  lightOnsetCount = 0U;
  lightLastOnset = 0U;
  lightPulseArmed = false;
  lightPulseActive = false;
  lightHighSamples = 0U;
  lightLowSamples = 0U;
  lightPulsePeakScore = 0.0f;
  lightRecentPulseStrength = 0.0f;
  lightCadenceHz = 0.0f;
  lightCadenceScore = 0.0f;
  lightFastScore = 0.0f;
}

void ImavLightRange::resetLightSpectrum()
{
  lightSpectralBins = {};
  lightSpectralBaseline = {};
  lightSpectralEnergy = 0.0f;
  lightSpectralScore = 0.0f;
  lightSpectralSnrDb = 0.0f;
  lightSpectralCoherence = 0.0f;
  lightSpectralRedFraction = 0.0f;
  lightSpectralFrequencyHz = 0.0f;
  lightHarmonicRatio = 0.0f;
  lightSpectralSamples = 0U;
  lightSpectralStartSample = 0U;
  lightSpectralLastSample = 0U;
  lightFlashScore = 0.0f;
}

void ImavLightRange::updateLightSpectrum(
  const std::array<float, 4U>& scaled, systime_t now)
{
  if ((lightSpectralLastSample != 0U) &&
      (chTimeDiffX(lightSpectralLastSample, now) >=
       TIME_MS2I(lightSpectralResetGapMs))) {
    resetLightSpectrum();
  }

  float samplePeriodSeconds = 0.0072f;
  if (lightSpectralLastSample != 0U) {
    samplePeriodSeconds = 1.0e-6f * static_cast<float>(TIME_I2US(
      chTimeDiffX(lightSpectralLastSample, now)));
  }
  // Preserve the original approximately 0.94 s DC and 9.4 s spectral time
  // constants when data-ready sampling runs near 139 Hz. A gap of 100 ms or
  // more has already reset the spectral state above.
  samplePeriodSeconds = std::clamp(
    samplePeriodSeconds, 0.0001f, 0.099f);
  const float dcAlpha = -std::expm1(-samplePeriodSeconds / 0.94f);
  const float spectralAlpha = -std::expm1(-samplePeriodSeconds / 9.4f);

  if (lightSpectralSamples == 0U) {
    lightSpectralBaseline = {scaled[0], scaled[1], scaled[2]};
    lightSpectralStartSample = now;
  }
  lightSpectralLastSample = now;
  ++lightSpectralSamples;

  std::array<float, 3U> ac = {};
  for (size_t channel = 0U; channel < ac.size(); ++channel) {
    lightSpectralBaseline[channel] += dcAlpha *
      (scaled[channel] - lightSpectralBaseline[channel]);
    ac[channel] = scaled[channel] - lightSpectralBaseline[channel];
  }

  // Red-sensitive, but deliberately broad enough for a different red LED
  // spectrum on the real motionSCOUT. The periodic color is checked again
  // from the complex RGB amplitudes below.
  const float redContrast = ac[0] - 0.5f * (ac[1] + ac[2]);
  lightSpectralEnergy += spectralAlpha *
    (redContrast * redContrast - lightSpectralEnergy);

  // All requested frequencies are integer multiples of 0.1 Hz, hence their
  // phase repeats every ten seconds. Work directly in system ticks so the
  // 32-bit TIME_I2US result cannot introduce a phase discontinuity every
  // 71.6 minutes.
  constexpr systime_t phasePeriod = TIME_S2I(10U);
  const systime_t phaseTicks = now % phasePeriod;
  const float baseAngle = twoPi *
    (static_cast<float>(phaseTicks) / static_cast<float>(phasePeriod));
  const float baseReal = std::cos(baseAngle);
  const float baseImag = -std::sin(baseAngle);
  float oscillatorReal = 1.0f;
  float oscillatorImag = 0.0f;
  size_t binIndex = 0U;
  for (uint8_t tenthHz = 1U;
       (tenthHz <= lightSpectralTenthsHz.back()) &&
       (binIndex < lightSpectralBins.size()); ++tenthHz) {
    const float nextReal =
      oscillatorReal * baseReal - oscillatorImag * baseImag;
    oscillatorImag =
      oscillatorReal * baseImag + oscillatorImag * baseReal;
    oscillatorReal = nextReal;
    if (tenthHz != lightSpectralTenthsHz[binIndex]) {
      continue;
    }

    LightSpectralBin& bin = lightSpectralBins[binIndex++];
    const auto update = [=](float value, float& real, float& imag) {
      real += spectralAlpha *
        (value * oscillatorReal - real);
      imag += spectralAlpha *
        (value * oscillatorImag - imag);
    };
    update(ac[0], bin.redReal, bin.redImag);
    update(ac[1], bin.greenReal, bin.greenImag);
    update(ac[2], bin.blueReal, bin.blueImag);
  }

  if ((lightSpectralSamples % lightSpectralUpdateDivider) != 0U) {
    updateLightCombinedScore();
    return;
  }

  const auto contrastPower = [](const LightSpectralBin& bin) {
    const float real = bin.redReal -
      0.5f * (bin.greenReal + bin.blueReal);
    const float imag = bin.redImag -
      0.5f * (bin.greenImag + bin.blueImag);
    return real * real + imag * imag;
  };

  size_t bestBinIndex = lightFirstTargetBin;
  float bestPower = 0.0f;
  for (size_t index = lightFirstTargetBin;
       index < lightFirstTargetBin + lightTargetBinCount; ++index) {
    const float power = contrastPower(lightSpectralBins[index]);
    if (power > bestPower) {
      bestPower = power;
      bestBinIndex = index;
    }
  }

  std::array<float, 14U> noiseAmplitudes = {};
  size_t noiseIndex = 0U;
  for (size_t index = 0U; index + 1U < lightSpectralBins.size(); ++index) {
    const int frequencyDistance = std::abs(
      static_cast<int>(lightSpectralTenthsHz[index]) -
      static_cast<int>(lightSpectralTenthsHz[bestBinIndex]));
    // Do not count the peak itself or the two immediately adjacent bins as
    // noise: a real frequency between DFT bins legitimately occupies both.
    if (frequencyDistance <= 1) {
      continue;
    }
    noiseAmplitudes[noiseIndex++] =
      std::sqrt(contrastPower(lightSpectralBins[index]));
  }
  chDbgAssert(noiseIndex == noiseAmplitudes.size(),
              "unexpected optical DFT noise-bin count");
  std::sort(noiseAmplitudes.begin(), noiseAmplitudes.end());
  const float noiseAmplitude =
    0.5f * (noiseAmplitudes[6] + noiseAmplitudes[7]);
  const float signalAmplitude = std::sqrt(bestPower);
  lightSpectralSnrDb = 20.0f * std::log10(
    (signalAmplitude + 1.0f) / (noiseAmplitude + 1.0f));
  lightSpectralCoherence = std::sqrt(std::clamp(
    2.0f * bestPower / (lightSpectralEnergy + 1.0f), 0.0f, 1.0f));
  lightSpectralFrequencyHz = 0.1f *
    static_cast<float>(lightSpectralTenthsHz[bestBinIndex]);

  const LightSpectralBin& bestBin = lightSpectralBins[bestBinIndex];
  const float redAmplitude = std::hypot(bestBin.redReal, bestBin.redImag);
  const float inverseRedAmplitude = 1.0f / (redAmplitude + 1.0f);
  const float greenProjection = std::max(0.0f,
    (bestBin.greenReal * bestBin.redReal +
     bestBin.greenImag * bestBin.redImag) * inverseRedAmplitude);
  const float blueProjection = std::max(0.0f,
    (bestBin.blueReal * bestBin.redReal +
     bestBin.blueImag * bestBin.redImag) * inverseRedAmplitude);
  lightSpectralRedFraction = redAmplitude /
    (redAmplitude + greenProjection + blueProjection + 1.0f);

  const float harmonicAmplitude = std::sqrt(
    contrastPower(lightSpectralBins[lightHarmonicBin]));
  lightHarmonicRatio = harmonicAmplitude / (signalAmplitude + 1.0f);

  const float snrScore = knee(lightSpectralSnrDb, 6.0f, 14.0f);
  const float coherenceScore =
    knee(lightSpectralCoherence, 0.04f, 0.14f);
  const float redScore =
    knee(lightSpectralRedFraction, 0.55f, 0.72f);
  const float support = std::min(
    static_cast<float>(TIME_I2MS(chTimeDiffX(
      lightSpectralStartSample, now))) / 10000.0f, 1.0f);
  // SNR and coherence are both mandatory evidence. Using their minimum keeps
  // a weak random peak from being inflated by a geometric mean. Color is a
  // softer qualifier because ground reflection and the real beacon lens can
  // alter its apparent spectrum substantially.
  lightSpectralScore = support * std::min(snrScore, coherenceScore) *
    (0.5f + 0.5f * redScore);
  updateLightCombinedScore();
}

void ImavLightRange::updateLightCombinedScore()
{
  if ((not lightFastTracking) &&
      (lightFastScore >= lightFastTrackingEnterScore)) {
    // Once the nearby beacon is unambiguous, spatial tracking must favor
    // current flash strength over the long-memory acquisition detector.
    lightFastTracking = true;
  }

  if (not lightFastTracking) {
    // Acquisition mode: retain maximum range, but allow a strong nearby
    // cadence to produce a sub-second attack.
    lightFlashScore = std::max(lightFastScore, lightSpectralScore);
    return;
  }

  // Tracking mode: gate the stale slow DFT with current proximity evidence.
  // The cadence freshness starts falling 450 ms after the last onset and is
  // zero at 900 ms, so lit follows the beacon spatially after an overflight.
  const float slowGate = knee(
    lightFastScore, lightFastTrackingExitScore,
    lightFastTrackingEnterScore);
  lightFlashScore = std::max(
    lightFastScore, lightSpectralScore * slowGate);

  if ((lightFastScore <= lightFastTrackingExitScore) &&
      (lightSpectralScore <= lightSlowTrackingExitScore)) {
    // Both paths have forgotten the beacon: permit a new long-range
    // acquisition without requiring a reboot.
    lightFastTracking = false;
  }
}

void ImavLightRange::appendLightOnset(systime_t now)
{
  if ((lightLastOnset != 0U) &&
      (chTimeDiffX(lightLastOnset, now) < TIME_MS2I(120U))) {
    return;
  }

  if (lightOnsetCount < lightOnsets.size()) {
    lightOnsets[lightOnsetCount++] = now;
  } else {
    for (size_t index = 1U; index < lightOnsets.size(); ++index) {
      lightOnsets[index - 1U] = lightOnsets[index];
    }
    lightOnsets.back() = now;
  }
  lightLastOnset = now;
}

void ImavLightRange::updateLightCadence(float instantScore, systime_t now)
{
  if ((lightLastSample != 0U) &&
      (chTimeDiffX(lightLastSample, now) >= TIME_MS2I(250U))) {
    clearLightCadence();
  }

  const bool high = instantScore >= 0.62f;
  const bool low = instantScore <= 0.25f;
  if (not lightPulseArmed) {
    lightHighSamples = 0U;
    lightLowSamples = low ? static_cast<uint8_t>(lightLowSamples + 1U) : 0U;
    if (lightLowSamples >= 2U) {
      lightPulseArmed = true;
      lightLowSamples = 0U;
    }
  } else if (not lightPulseActive) {
    lightLowSamples = 0U;
    lightHighSamples = high
      ? static_cast<uint8_t>(lightHighSamples + 1U) : 0U;
    if (lightHighSamples >= 2U) {
      lightPulseActive = true;
      lightHighSamples = 0U;
      lightPulsePeakScore = instantScore;
      ++lightPulses;
      appendLightOnset(now);
    }
  } else {
    lightHighSamples = 0U;
    lightPulsePeakScore = std::max(lightPulsePeakScore, instantScore);
    lightLowSamples = low ? static_cast<uint8_t>(lightLowSamples + 1U) : 0U;
    if (lightLowSamples >= 2U) {
      lightPulseActive = false;
      lightLowSamples = 0U;
      lightRecentPulseStrength = lightPulsePeakScore;
      lightPulsePeakScore = 0.0f;
    }
  }

  if (lightLastOnset == 0U) {
    lightCadenceHz = 0.0f;
    lightCadenceScore = 0.0f;
    lightFastScore = 0.0f;
    return;
  }

  const uint32_t onsetAgeMs = TIME_I2MS(
    chTimeDiffX(lightLastOnset, now));
  if (onsetAgeMs >= 1500U) {
    clearLightCadence();
    return;
  }

  const uint8_t intervalCount = lightOnsetCount > 0U
    ? static_cast<uint8_t>(lightOnsetCount - 1U) : 0U;
  if (intervalCount == 0U) {
    lightCadenceHz = 0.0f;
    lightCadenceScore = 0.0f;
    lightFastScore = 0.0f;
    return;
  }

  float periodScoreSum = 0.0f;
  float periodMsSum = 0.0f;
  for (uint8_t index = 1U; index < lightOnsetCount; ++index) {
    const float periodMs = static_cast<float>(TIME_I2MS(
      chTimeDiffX(lightOnsets[index - 1U], lightOnsets[index])));
    const float prealarmError = (periodMs - 500.0f) / 90.0f;
    const float alarmError = (periodMs - 333.333f) / 80.0f;
    const float prealarmScore =
      1.0f / (1.0f + prealarmError * prealarmError);
    const float alarmScore =
      1.0f / (1.0f + alarmError * alarmError);
    periodScoreSum += std::max(prealarmScore, alarmScore);
    periodMsSum += periodMs;
  }

  const float meanPeriodMs = periodMsSum / intervalCount;
  lightCadenceHz = 1000.0f / meanPeriodMs;
  // Two coherent intervals mean three observed flashes: enough evidence for
  // the proximity path, while still rejecting an isolated pair of edges.
  const float support = std::min(
    static_cast<float>(intervalCount) / 2.0f, 1.0f);
  const float freshness = onsetAgeMs <= 450U ? 1.0f :
    std::max(0.0f,
      (900.0f - static_cast<float>(onsetAgeMs)) / 450.0f);
  lightCadenceScore =
    (periodScoreSum / intervalCount) * support * freshness;
  const float strength = lightPulseActive
    ? std::max(lightRecentPulseStrength, lightPulsePeakScore)
    : lightRecentPulseStrength;
  lightFastScore =
    lightCadenceScore * (0.5f + 0.5f * strength);
}

void ImavLightRange::processLightMeasurement(
  const std::array<uint32_t, 4U>& adcCodes, bool overloaded,
  systime_t now)
{
  lightRed = adcCodes[0];
  lightGreen = adcCodes[1];
  lightBlue = adcCodes[2];
  lightWide = adcCodes[3];
  ++lightSamples;

  // TI's data-sheet scaling makes a D65 white source approximately R=G=B.
  const std::array<float, 4U> scaled = {
    2.4f * static_cast<float>(adcCodes[0]),
    static_cast<float>(adcCodes[1]),
    1.3f * static_cast<float>(adcCodes[2]),
    static_cast<float>(adcCodes[3]),
  };

  if (overloaded) {
    if (not lightOverloadActive) {
      ++lightSaturations;
    }
    lightOverloadActive = true;
    lightRedRatio = 0.0f;
    lightRelativeAc = 0.0f;
    lightInstantScore = 0.0f;
    updateLightCadence(0.0f, now);
    resetLightSpectrum();
    lightLastSample = now;
    publishLightState();
    publishLightDebugSample(overloaded);
    return;
  }
  lightOverloadActive = false;

  if (not lightBaselineValid) {
    lightBaseline = scaled;
    lightBaselineValid = true;
    lightRedRatio = 0.0f;
    lightRelativeAc = 0.0f;
    lightInstantScore = 0.0f;
    updateLightCadence(0.0f, now);
    updateLightSpectrum(scaled, now);
    lightLastSample = now;
    publishLightState();
    publishLightDebugSample(overloaded);
    return;
  }

  std::array<float, 4U> delta = {};
  std::array<float, 4U> positiveDelta = {};
  for (size_t channel = 0U; channel < optChannelCount; ++channel) {
    delta[channel] = scaled[channel] - lightBaseline[channel];
    positiveDelta[channel] = std::max(delta[channel], 0.0f);
  }

  const float positiveRgb =
    positiveDelta[0] + positiveDelta[1] + positiveDelta[2];
  lightRedRatio = positiveRgb > 1.0f
    ? positiveDelta[0] / positiveRgb : 0.0f;
  const float normalizer = lightBaseline[0] + 1024.0f;
  lightRelativeAc = positiveDelta[0] / normalizer;
  const float absoluteAc = std::abs(delta[0]) / normalizer;
  const float noise = lightNoiseValid ? lightNoiseFloor : 0.002f;

  // These provisional knees are deliberately exposed through debug values;
  // measurements on the competition beacon must be used to tune them.
  const float absoluteScore = knee(positiveDelta[0], 256.0f, 4096.0f);
  const float relativeScore = knee(lightRelativeAc, 0.01f, 0.25f);
  const float riseScore = knee(
    lightRelativeAc / (noise + 0.002f), 3.0f, 12.0f);
  const float redScore = knee(lightRedRatio, 0.42f, 0.68f);
  const float signalScore = std::max(relativeScore, 0.8f * riseScore);
  lightInstantScore = redScore * signalScore *
    (0.4f + 0.6f * absoluteScore);
  updateLightCadence(lightInstantScore, now);

  // Do not let a red flash pull the slow ambient estimate upward. A falling
  // illumination is followed faster so a shadow does not create a long bias.
  if ((not lightPulseActive) && (lightInstantScore < 0.20f)) {
    for (size_t channel = 0U; channel < optChannelCount; ++channel) {
      const float baselineAlpha = delta[channel] < 0.0f
        ? (1.0f / 16.0f) : (1.0f / 256.0f);
      lightBaseline[channel] = std::max(
        0.0f, lightBaseline[channel] + delta[channel] * baselineAlpha);
    }
    if (not lightNoiseValid) {
      lightNoiseFloor = absoluteAc;
      lightNoiseValid = true;
    } else {
      const float noiseAlpha = absoluteAc < lightNoiseFloor
        ? (1.0f / 16.0f) : (1.0f / 256.0f);
      lightNoiseFloor +=
        (absoluteAc - lightNoiseFloor) * noiseAlpha;
    }
  }

  // The spectral detector is deliberately updated after the legacy pulse
  // detector, so the navigation score is the locally noise-normalized 3 Hz
  // score while the threshold/cadence metrics remain available for tuning.
  updateLightSpectrum(scaled, now);
  lightLastSample = now;
  publishLightState();
  publishLightDebugSample(overloaded);
}

bool ImavLightRange::sampleLight(systime_t sampleTime)
{
  if (not lightRunning) {
    return true;
  }
  if (not readLightBlock(optRegChannel0Msb, optResultBytes)) {
    return false;
  }

  std::array<uint32_t, optChannelCount> adcCodes = {};
  std::array<uint8_t, optChannelCount> counters = {};
  for (size_t channel = 0U; channel < optChannelCount; ++channel) {
    const size_t offset = channel * 4U;
    const Opt4060ChannelSample sample = decodeOpt4060Channel(
      readBigEndian16(&lightRx[offset]),
      readBigEndian16(&lightRx[offset + 2U]));
    if (not sample.valid) {
      ++lightReadErrors;
      return false;
    }
    adcCodes[channel] = sample.adcCode;
    counters[channel] = sample.counter;
  }

  if (lightCountersValid) {
    for (size_t channel = 0U; channel < optChannelCount; ++channel) {
      if (counters[channel] == lightCounters[channel]) {
        // The four channels convert sequentially. Wait until every channel
        // has advanced instead of mixing a new partial cycle with an old one.
        if (++lightStalePolls < 3U) {
          return true;
        }
        ++lightReadErrors;
        return false;
      }
    }
  }
  lightStalePolls = 0U;
  lightCounters = counters;
  lightCountersValid = true;

  uint16_t status = 0U;
  if (not readLightRegister(optRegStatus, status)) {
    return false;
  }
  processLightMeasurement(
    adcCodes, (status & optStatusOverload) != 0U,
    sampleTime);
  return true;
}

int32_t ImavLightRange::tofBusInit()
{
  return active != nullptr ? 0 : -1;
}

int32_t ImavLightRange::tofBusDeinit()
{
  return 0;
}

int32_t ImavLightRange::tofGetTick()
{
  return static_cast<int32_t>(TIME_I2MS(chVTGetSystemTimeX()));
}

int32_t ImavLightRange::tofBusWrite(uint16_t address, uint8_t *data,
                                    uint16_t length)
{
  if ((active == nullptr) || (data == nullptr)) {
    return -1;
  }

  // The ST platform first writes a two-byte register address and then invokes
  // ReadReg. Defer that address-only write so ReadReg can issue one atomic
  // write/repeated-start/read transaction on the shared bus.
  if (length == 2U) {
    active->pendingTofRegister[0] = data[0];
    active->pendingTofRegister[1] = data[1];
    active->pendingTofAddress = address;
    active->pendingTofRead = true;
    return 0;
  }

  active->pendingTofRead = false;
  return active->tofTransfer(address, data, length, 0U) == MSG_OK ? 0 : -1;
}

int32_t ImavLightRange::tofBusRead(uint16_t address, uint8_t *data,
                                   uint16_t length)
{
  if ((active == nullptr) || (data == nullptr) ||
      (not active->pendingTofRead) ||
      (active->pendingTofAddress != address)) {
    return -1;
  }

  active->pendingTofRead = false;
  const msg_t result = active->tofTransfer(
    address, active->pendingTofRegister,
    sizeof(active->pendingTofRegister), length);
  if (result != MSG_OK) {
    return -1;
  }
  std::memcpy(data, active->tofRx, length);
  return 0;
}

msg_t ImavLightRange::tofTransfer(uint16_t address, const uint8_t *tx,
                                  size_t txLength, size_t rxLength)
{
  if ((txLength > sizeof(tofTx)) || (rxLength > sizeof(tofRx)) ||
      (address > 0xFEU)) {
    ++rangeErrors;
    return MSG_RESET;
  }

  if (txLength != 0U) {
    if (tx != tofTx) {
      std::memcpy(tofTx, tx, txLength);
    }
  }
  i2cAcquireBus(&ExternalI2CD);
  const msg_t result = i2cMasterTransmitTimeout(
    &ExternalI2CD, static_cast<i2caddr_t>(address >> 1U),
    tofTx, txLength, tofRx, rxLength, TIME_MS2I(50U));
  if (result != MSG_OK) {
    recordI2cFailure(true, result);
  }
  i2cReleaseBus(&ExternalI2CD);
  return result;
}

bool ImavLightRange::initializeRange()
{
  if (not timeOfFlightEnabled) {
    return false;
  }

  std::memset(&rangeDevice, 0, sizeof(rangeDevice));
  pendingTofRead = false;
  VL53L4CX_IO_t io = {
    .Init = &ImavLightRange::tofBusInit,
    .DeInit = &ImavLightRange::tofBusDeinit,
    .Address = VL53L4CX_DEVICE_ADDRESS,
    .WriteReg = &ImavLightRange::tofBusWrite,
    .ReadReg = &ImavLightRange::tofBusRead,
    .GetTick = &ImavLightRange::tofGetTick,
  };

  uint32_t id = 0U;
  if ((VL53L4CX_RegisterBusIO(&rangeDevice, &io) != VL53L4CX_OK) ||
      (VL53L4CX_ReadID(&rangeDevice, &id) != VL53L4CX_OK) ||
      (id != VL53L4CX_ID)) {
    ++rangeErrors;
    rangeDeviceId = id;
    rangeAvailable = false;
    return false;
  }

  if (VL53L4CX_Init(&rangeDevice) != VL53L4CX_OK) {
    ++rangeErrors;
    rangeDeviceId = id;
    rangeAvailable = false;
    return false;
  }

  VL53L4CX_ProfileConfig_t profile = {
    .RangingProfile = VL53L4CX_PROFILE_LONG,
    .TimingBudget = rangeTimingBudgetMs,
    .Frequency = 0U,
    .EnableAmbient = 1U,
    .EnableSignal = 1U,
  };
  if (VL53L4CX_ConfigProfile(&rangeDevice, &profile) != VL53L4CX_OK) {
    ++rangeErrors;
    rangeDeviceId = id;
    rangeAvailable = false;
    return false;
  }

  rangeDeviceId = id;
  rangeAvailable = true;
  rangeConsecutiveErrors = 0U;
  publishAvailability();
  return true;
}

bool ImavLightRange::measureRange()
{
  VL53L4CX_Result_t result = {};
  int32_t status = VL53L4CX_Start(
    &rangeDevice, VL53L4CX_MODE_ASYNC_ONESHOT);
  if (status == VL53L4CX_OK) {
    const systime_t start = chVTGetSystemTimeX();
    do {
      status = VL53L4CX_GetDistance(&rangeDevice, &result);
      if (status != VL53L4CX_TIMEOUT) {
        break;
      }
      chThdSleepMilliseconds(1U);
    } while (chTimeDiffX(start, chVTGetSystemTimeX()) < TIME_MS2I(100U));
  }
  if (rangeDevice.IsRanging != 0U) {
    const int32_t stopStatus = VL53L4CX_Stop(&rangeDevice);
    if ((status == VL53L4CX_OK) && (stopStatus != VL53L4CX_OK)) {
      status = stopStatus;
    }
  }
  if (status != VL53L4CX_OK) {
    ++rangeErrors;
    rangeValid = false;
    chSysLock();
    published.rangeValid = false;
    published.rangeErrors = rangeErrors;
    chSysUnlock();
    publishAvailability();
    return false;
  }

  size_t selected = VL53L4CX_NB_TARGET_PER_ZONE;
  uint32_t nearest = std::numeric_limits<uint32_t>::max();
  const VL53L4CX_TargetResult_t& zone = result.ZoneResult[0];
  const size_t targetCount = std::min(
    static_cast<size_t>(zone.NumberOfTargets),
    static_cast<size_t>(VL53L4CX_NB_TARGET_PER_ZONE));
  for (size_t target = 0U; target < targetCount; ++target) {
    if ((zone.Status[target] == 0U) &&
        (zone.Distance[target] < nearest)) {
      nearest = zone.Distance[target];
      selected = target;
    }
  }

  rangeValid = selected < VL53L4CX_NB_TARGET_PER_ZONE;
  if (rangeValid) {
    rangeMm = static_cast<uint16_t>(
      std::min(nearest,
               static_cast<uint32_t>(
                 std::numeric_limits<uint16_t>::max())));
    rangeSignalKcps = zone.Signal[selected];
    rangeAmbientKcps = zone.Ambient[selected];
  } else {
    rangeMm = static_cast<uint16_t>(rangeMaximumMetres * 1000.0f);
    rangeSignalKcps = 0.0f;
    rangeAmbientKcps = 0.0f;
  }

  ++rangeMeasurements;
  rangeLastSample = chVTGetSystemTimeX();
  chSysLock();
  published.rangeAvailable = true;
  published.rangeValid = rangeValid;
  published.rangeDeviceId = rangeDeviceId;
  published.rangeMm = rangeMm;
  published.rangeSignalKcps = rangeSignalKcps;
  published.rangeAmbientKcps = rangeAmbientKcps;
  published.rangeMeasurements = rangeMeasurements;
  published.rangeErrors = rangeErrors;
  published.rangeLastSample = rangeLastSample;
  chSysUnlock();
  publishRange(rangeValid);
  return true;
}

void ImavLightRange::publishRange(bool valid)
{
  uavcan_equipment_range_sensor_Measurement message = {};
  message.timestamp.usec =
    static_cast<uint64_t>(TIME_I2US(chVTGetSystemTimeX()));
  message.sensor_id = 0U;
  // The suspended module points down under gravity, but its yaw/roll relative
  // to the aircraft body is not fixed; do not publish a false body attitude.
  message.beam_orientation_in_body_frame.orientation_defined = false;
  message.field_of_view = rangeFieldOfViewRad;
  message.sensor_type =
    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
  message.reading_type = valid
    ? UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE
    : UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
  message.range = valid
    ? static_cast<float>(rangeMm) * 0.001f : rangeMaximumMetres;
  node.sendBroadcast(message, CANARD_TRANSFER_PRIORITY_MEDIUM);
}

uint32_t ImavLightRange::nextRangeIntervalMs()
{
  // Walking the phase prevents a periodic beacon flash from always falling
  // into the short light-acquisition gap created by ToF ranging.
  static constexpr std::array<int8_t, 8U> jitterMs = {
    -11, 7, -3, 13, -7, 3, 0, 9,
  };
  const int32_t interval =
    static_cast<int32_t>(rangePeriodMs) +
    jitterMs[rangeJitterIndex++ % jitterMs.size()];
  return static_cast<uint32_t>(std::max(interval, int32_t{100}));
}

[[noreturn]] void ImavLightRange::run()
{
  systime_t now = chVTGetSystemTimeX();
  systime_t lastLightRetry = now;
  systime_t lastRangeRetry = now;
  systime_t lastRangeStart = now;
  uint32_t rangeInterval = 100U;

  while (true) {
    if (lightRestartPending) {
      bool rangeStopped = true;
      if (rangeDevice.IsRanging != 0U) {
        rangeStopped =
          VL53L4CX_Stop(&rangeDevice) == VL53L4CX_OK;
      }
      if (not rangeStopped) {
        // Never overlap an unconfirmed 940 nm ranging operation with the
        // light detector. Retry the stop while leaving the OPT4060 paused.
        rangeAvailable = false;
        rangeValid = false;
        publishAvailability();
        lastRangeRetry = chVTGetSystemTimeX();
        chThdSleepMilliseconds(optPollPeriodMs);
        continue;
      }

      lightRestartPending = false;
      if (not startLight()) {
        lightAvailable = false;
        publishAvailability();
        lastLightRetry = chVTGetSystemTimeX();
      } else {
        lightAvailable = true;
        publishAvailability();
      }
    }

    if (lightRunning) {
      // INT_CFG=3 produces a falling edge only after a coherent RGBW group is
      // ready. A timeout retains a degraded polling path so a broken INT wire
      // cannot make the optical detector fail silently.
      const msg_t ready = palWaitLineTimeout(
        LINE_OPT4060_INT, TIME_MS2I(optDataReadyTimeoutMs));
      const systime_t sampleTime = chVTGetSystemTimeX();
      if (ready == MSG_TIMEOUT) {
        // Node::infoCb is routed to DebugTrace by the application and repeated
        // identical messages are rate-limited there, avoiding serial floods.
        node.infoCb("WARN: OPT4060 INT timeout, polling fallback");
      } else if (ready != MSG_OK) {
        node.infoCb("WARN: OPT4060 INT wait reset, polling fallback");
      }

      if (sampleLight(sampleTime)) {
        lightConsecutiveErrors = 0U;
      } else if (++lightConsecutiveErrors >= 3U) {
        lightAvailable = false;
        publishAvailability();
        lastLightRetry = chVTGetSystemTimeX();
      }
    }

    now = chVTGetSystemTimeX();
    if ((not lightAvailable) &&
        (chTimeDiffX(lastLightRetry, now) >=
         TIME_MS2I(sensorRetryPeriodMs))) {
      if (initializeLight()) {
        node.infoCb("IMAV light recovered: OPT4060 addr=0x%02x",
                    lightAddress);
      }
      lastLightRetry = chVTGetSystemTimeX();
    }

    now = chVTGetSystemTimeX();
    const bool rangeDue = timeOfFlightEnabled && rangeAvailable &&
      (chTimeDiffX(lastRangeStart, now) >= TIME_MS2I(rangeInterval));
    const bool rangeRetryDue = timeOfFlightEnabled && (not rangeAvailable) &&
      (chTimeDiffX(lastRangeRetry, now) >=
       TIME_MS2I(sensorRetryPeriodMs));
    if (rangeDue || rangeRetryDue) {
      lastRangeStart = now;
      bool lightPaused = not lightRunning;
      if (lightRunning) {
        (void) sampleLight(chVTGetSystemTimeX());
        lightPaused = stopLight();
        if (lightPaused) {
          lightRestartPending = true;
          ++lightGaps;
          publishLightState();
        }
      }

      if (lightPaused) {
        if (rangeDue) {
          if (measureRange()) {
            rangeConsecutiveErrors = 0U;
          } else if (++rangeConsecutiveErrors >= 3U) {
            rangeAvailable = false;
            publishAvailability();
            lastRangeRetry = chVTGetSystemTimeX();
          }
        } else {
          rangeAvailable = initializeRange();
          lastRangeRetry = chVTGetSystemTimeX();
          if (rangeAvailable) {
            node.infoCb("IMAV range recovered: VL53L4CX id=0x%04lx",
                        static_cast<unsigned long>(rangeDeviceId));
          }
        }
      }

      rangeInterval = nextRangeIntervalMs();
    }

    if (not lightRunning) {
      // Avoid a busy loop while the OPT4060 is absent or awaiting recovery.
      chThdSleepMilliseconds(optPollPeriodMs);
    }
  }
}

#endif // USE_IMAV_ROLE
