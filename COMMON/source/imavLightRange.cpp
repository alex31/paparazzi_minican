/**
 * @file imavLightRange.cpp
 * @brief TCS3410 FIFO flash detection and VL53L4CX ground ranging.
 */

#include "roleConf.h"

#if USE_IMAV_ROLE

#include "imavLightRange.hpp"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
  constexpr uint8_t tcs3410Address0 = 0x39U;
  constexpr uint8_t tcs3410Address1 = 0x49U;
  constexpr uint8_t tcs3410ExpectedId = 0x5CU;

  constexpr uint8_t tcsRegModChannelControl = 0x40U;
  constexpr uint8_t tcsRegEnable = 0x80U;
  constexpr uint8_t tcsRegMeasMode1 = 0x82U;
  constexpr uint8_t tcsRegSampleTime0 = 0x83U;
  constexpr uint8_t tcsRegSampleTime1 = 0x84U;
  constexpr uint8_t tcsRegFdSamples0 = 0x87U;
  constexpr uint8_t tcsRegFdSamples1 = 0x88U;
  constexpr uint8_t tcsRegId = 0x92U;
  constexpr uint8_t tcsRegControl = 0xB1U;
  constexpr uint8_t tcsRegSequencerFd01 = 0xCFU;
  constexpr uint8_t tcsRegSequencerAlsFd2 = 0xD0U;
  constexpr uint8_t tcsRegSequencerResidual01 = 0xD2U;
  constexpr uint8_t tcsRegSequencerResidual2Wait = 0xD3U;
  constexpr uint8_t tcsRegStep0Gain01 = 0xD4U;
  constexpr uint8_t tcsRegStep0SmuxLow = 0xDCU;
  constexpr uint8_t tcsRegStep0SmuxHigh = 0xDDU;
  constexpr uint8_t tcsRegFifoConfig0 = 0xF9U;
  constexpr uint8_t tcsRegFifoConfig1 = 0xFAU;
  constexpr uint8_t tcsRegFifoConfig2 = 0xFBU;
  constexpr uint8_t tcsRegFifoThreshold = 0xFCU;
  constexpr uint8_t tcsRegFifoStatus0 = 0xFDU;
  constexpr uint8_t tcsRegFifoData = 0xFFU;

  constexpr uint8_t tcsEnablePower = 1U << 0U;
  constexpr uint8_t tcsEnableFlicker = 1U << 6U;
  constexpr uint8_t tcsControlFifoClear = 1U << 1U;
  constexpr uint8_t tcsFifoOverflow = 1U << 7U;
  constexpr uint8_t tcsFifoUnderflow = 1U << 6U;
  constexpr size_t tcsFifoCapacity = 512U;
  constexpr size_t tcsBytesPerSample = 2U;
  constexpr uint32_t tcsPollPeriodMs = 4U;

  // SAMPLE_TIME=89 gives 125 us, i.e. 8 ksample/s. Modulator 0 receives
  // both F-filtered photodiodes (PD0 and PD2). ALS and residual measurements
  // are disabled, so the FIFO contains only little-endian 16-bit samples.
  constexpr uint8_t tcsSampleTime8Khz = 89U;
  constexpr uint8_t tcsStep0Gain16x = 5U;

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
                               uint32_t periodMs)
  : node(node_),
    lightAddress(address),
    rangePeriodMs(std::clamp(
      periodMs, uint32_t{100U}, uint32_t{1000U}))
{
  chDbgAssert(active == nullptr, "single IMAV light/range instance expected");
  active = this;
  published.lightAddress = address;
}

void ImavLightRange::initialize()
{
  // Ref-SPAD initialization can emit 940 nm light. Configure it before the
  // TCS3410 starts, so the two sensors are never acquiring simultaneously.
  rangeAvailable = initializeRange();
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
  published.lightFifoOverflows = lightFifoOverflows;
  published.lightGaps = lightGaps;
  published.rangeErrors = rangeErrors;
  chSysUnlock();
}

void ImavLightRange::publishLightState()
{
  chSysLock();
  published.lightRaw = lightRaw;
  published.lightRelativeAc = lightRelativeAc;
  published.lightFlashScore = lightFlashHold;
  published.lightSamples = lightSamples;
  published.lightPulses = lightPulses;
  published.lightSaturations = lightSaturations;
  published.lightFifoOverflows = lightFifoOverflows;
  published.lightReadErrors = lightReadErrors;
  published.lightBusRecoveries = lightBusRecoveries;
  published.lightGaps = lightGaps;
  published.lightLastSample = lightLastSample;
  chSysUnlock();
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

bool ImavLightRange::readLightRegister(uint8_t reg, uint8_t& value)
{
  lightTx[0] = reg;
  if (lightTransfer(1U, 1U) != MSG_OK) {
    return false;
  }
  value = lightRx[0];
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

bool ImavLightRange::writeLightRegister(uint8_t reg, uint8_t value)
{
  lightTx[0] = reg;
  lightTx[1] = value;
  return lightTransfer(2U, 0U) == MSG_OK;
}

bool ImavLightRange::initializeLight()
{
  lightAvailable = false;
  const uint8_t preferred =
    ((lightAddress == tcs3410Address0) || (lightAddress == tcs3410Address1))
    ? lightAddress : tcs3410Address0;
  const std::array<uint8_t, 2U> candidates = {
    preferred,
    preferred == tcs3410Address0 ? tcs3410Address1 : tcs3410Address0,
  };

  uint8_t id = 0U;
  bool found = false;
  for (const uint8_t candidate : candidates) {
    lightAddress = candidate;
    if (readLightRegister(tcsRegId, id) && (id == tcs3410ExpectedId)) {
      found = true;
      break;
    }
  }
  if (not found) {
    publishAvailability();
    return false;
  }

  // Do not claim that acquisition is stopped until the sensor acknowledges
  // FDEN=0; this invariant gates every ToF measurement.
  if (not writeLightRegister(tcsRegEnable, 0U)) {
    publishAvailability();
    return false;
  }
  lightRunning = false;
  // The sequence follows the public TCS3410 data sheet and AN001059. In
  // particular, 0x11/0xF0 is ams OSRAM's SMUX setting for both F diodes.
  const bool configured =
    writeLightRegister(tcsRegModChannelControl, 0x06U) &&
    writeLightRegister(tcsRegMeasMode1, 0x0CU) &&
    writeLightRegister(tcsRegSampleTime0, tcsSampleTime8Khz) &&
    writeLightRegister(tcsRegSampleTime1, 0U) &&
    writeLightRegister(tcsRegFdSamples0, 0U) &&
    writeLightRegister(tcsRegFdSamples1, 0x80U) &&
    writeLightRegister(tcsRegSequencerFd01, 0x01U) &&
    writeLightRegister(tcsRegSequencerAlsFd2, 0U) &&
    writeLightRegister(tcsRegSequencerResidual01, 0U) &&
    writeLightRegister(tcsRegSequencerResidual2Wait, 0U) &&
    writeLightRegister(tcsRegStep0Gain01,
                       static_cast<uint8_t>(0x80U | tcsStep0Gain16x)) &&
    writeLightRegister(tcsRegStep0SmuxLow, 0x11U) &&
    writeLightRegister(tcsRegStep0SmuxHigh, 0xF0U) &&
    writeLightRegister(tcsRegFifoConfig0, 0x0FU) &&
    writeLightRegister(tcsRegFifoConfig1, 0x0FU) &&
    writeLightRegister(tcsRegFifoConfig2, 0x0FU) &&
    writeLightRegister(tcsRegFifoThreshold, 0x1FU) &&
    writeLightRegister(tcsRegEnable, tcsEnablePower);
  if (not configured) {
    publishAvailability();
    return false;
  }

  chThdSleepMilliseconds(1U);
  if (not startLight()) {
    publishAvailability();
    return false;
  }

  lightBaselineValid = false;
  lightNoiseValid = false;
  lightPulseActive = false;
  lightFlashHold = 0.0f;
  lightConsecutiveErrors = 0U;
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
  if (not writeLightRegister(tcsRegControl, tcsControlFifoClear)) {
    return false;
  }
  if (not writeLightRegister(tcsRegEnable,
                             tcsEnablePower | tcsEnableFlicker)) {
    return false;
  }
  lightRunning = true;
  return true;
}

bool ImavLightRange::stopLight()
{
  if (not writeLightRegister(tcsRegEnable, tcsEnablePower)) {
    return false;
  }
  lightRunning = false;
  // FDEN is already off, which is the safety condition for ranging. A failed
  // FIFO clear is retried by startLight() before sampling resumes.
  (void) writeLightRegister(tcsRegControl, tcsControlFifoClear);
  return true;
}

void ImavLightRange::processLightSamples(const uint8_t *bytes, size_t length)
{
  for (size_t offset = 0U; (offset + 1U) < length;
       offset += tcsBytesPerSample) {
    const uint16_t sample =
      static_cast<uint16_t>(bytes[offset]) |
      static_cast<uint16_t>(bytes[offset + 1U]) << 8U;
    lightRaw = sample;
    ++lightSamples;

    // The TCS3410 encodes analogue saturation as all ones. It means the
    // optical path is temporarily blind, not that a beacon edge was found.
    if (sample == std::numeric_limits<uint16_t>::max()) {
      ++lightSaturations;
      lightFlashHold *= 0.9998f;
      continue;
    }

    if (not lightBaselineValid) {
      lightBaseline = static_cast<float>(sample);
      lightBaselineValid = true;
      continue;
    }

    const float delta = static_cast<float>(sample) - lightBaseline;
    const float normalizer = lightBaseline + 16.0f;
    const float positiveAc = std::max(delta, 0.0f) / normalizer;
    const float absoluteAc = std::abs(delta) / normalizer;
    const float noise = lightNoiseValid ? lightNoiseFloor : 0.0005f;
    const float absoluteScore = knee(positiveAc, 0.015f, 0.30f);
    const float riseScore = knee(positiveAc / (noise + 0.0005f),
                                 3.0f, 12.0f);
    const float instantScore = absoluteScore * riseScore;

    if (lightPulseActive) {
      if (instantScore <= 0.20f) {
        lightPulseActive = false;
      }
    } else if (instantScore >= 0.65f) {
      lightPulseActive = true;
      ++lightPulses;
    }

    // Do not let a flash pull the slow illumination estimate upward.
    if ((not lightPulseActive) && (instantScore < 0.20f)) {
      const float baselineAlpha = delta < 0.0f
        ? (1.0f / 128.0f) : (1.0f / 2048.0f);
      lightBaseline =
        std::max(0.0f, lightBaseline + delta * baselineAlpha);
      if (not lightNoiseValid) {
        lightNoiseFloor = absoluteAc;
        lightNoiseValid = true;
      } else {
        const float noiseAlpha = absoluteAc < lightNoiseFloor
          ? (1.0f / 64.0f) : (1.0f / 1024.0f);
        lightNoiseFloor +=
          (absoluteAc - lightNoiseFloor) * noiseAlpha;
      }
    }

    // About 0.6 s e-folding at 8 ksample/s: a one-sample flash remains
    // observable by the lower-rate debug and fusion code.
    lightFlashHold = std::max(instantScore, lightFlashHold * 0.9998f);
    lightRelativeAc = positiveAc;
  }

  lightLastSample = chVTGetSystemTimeX();
  publishLightState();
}

bool ImavLightRange::drainLightFifo()
{
  if (not lightRunning) {
    return true;
  }

  constexpr size_t maxChunks =
    tcsFifoCapacity / sizeof(lightRx);
  for (size_t chunk = 0U; chunk < maxChunks; ++chunk) {
    if (not readLightBlock(tcsRegFifoStatus0, 2U)) {
      return false;
    }

    const uint8_t status1 = lightRx[1];
    const size_t level =
      (static_cast<size_t>(lightRx[0]) << 2U) |
      static_cast<size_t>(status1 & 0x03U);
    if ((status1 & (tcsFifoOverflow | tcsFifoUnderflow)) != 0U) {
      if ((status1 & tcsFifoOverflow) != 0U) {
        ++lightFifoOverflows;
      }
      lightBaselineValid = false;
      (void) writeLightRegister(tcsRegControl, tcsControlFifoClear);
      publishLightState();
      return true;
    }
    if (level == 0U) {
      return true;
    }
    if ((level & 1U) != 0U) {
      ++lightReadErrors;
      lightBaselineValid = false;
      (void) writeLightRegister(tcsRegControl, tcsControlFifoClear);
      publishLightState();
      return true;
    }

    const size_t readLength =
      std::min(level, sizeof(lightRx));
    if (not readLightBlock(tcsRegFifoData, readLength)) {
      return false;
    }
    processLightSamples(lightRx, readLength);
  }
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
    std::memcpy(tofTx, tx, txLength);
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
        // light detector. Retry the stop while leaving the TCS3410 paused.
        rangeAvailable = false;
        rangeValid = false;
        publishAvailability();
        lastRangeRetry = chVTGetSystemTimeX();
        chThdSleepMilliseconds(tcsPollPeriodMs);
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
      if (drainLightFifo()) {
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
        node.infoCb("IMAV light recovered: TCS3410 addr=0x%02x",
                    lightAddress);
      }
      lastLightRetry = chVTGetSystemTimeX();
    }

    now = chVTGetSystemTimeX();
    const bool rangeDue = rangeAvailable &&
      (chTimeDiffX(lastRangeStart, now) >= TIME_MS2I(rangeInterval));
    const bool rangeRetryDue = (not rangeAvailable) &&
      (chTimeDiffX(lastRangeRetry, now) >=
       TIME_MS2I(sensorRetryPeriodMs));
    if (rangeDue || rangeRetryDue) {
      lastRangeStart = now;
      bool lightPaused = not lightRunning;
      if (lightRunning) {
        (void) drainLightFifo();
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

    chThdSleepMilliseconds(tcsPollPeriodMs);
  }
}

#endif // USE_IMAV_ROLE
