/** @file Scheduled RGBW acquisition with periodic or on-change publication. */
#include "roleConf.h"
#if USE_OPT4060_ROLE

#include "opt4060Role.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "I2C_periph.hpp"
#include <cmath>
#include <utility>

namespace {
  const ioline_t interruptLine = PAL_LINE(GPIOA, 8U);
  constexpr uint8_t resultRegister = 0x00U;
  constexpr uint8_t configurationRegister = 0x0AU;
  constexpr uint8_t configuration2Register = 0x0BU;
  constexpr uint8_t statusRegister = 0x0CU;
  constexpr uint8_t idRegister = 0x11U;
  constexpr uint16_t powerDown = 0x3008U;
  constexpr uint16_t modeMask = 0x0030U;
  // INT pulses after all four channels complete; burst reads enabled.
  constexpr uint16_t configuration2 = 0x801DU;

  uint64_t nowUs() {
    return chVTGetTimeStamp() * 1'000'000ULL / CH_CFG_ST_FREQUENCY;
  }
}

DeviceStatus Opt4060Role::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  return DeviceStatus(DeviceStatus::OPT4060);
}

DeviceStatus Opt4060Role::start(UAVCAN::Node& node)
{
  m_node = &node;
  const uint32_t frequency = param_cget<"bus.i2c.frequency_khz">();
  if (frequency != 100U && frequency != 400U) {
    return DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::I2C_FREQ_INVALID,
                        static_cast<uint16_t>(frequency));
  }
  const auto settings = Opt4060Timing::configure(
    param_cget<"role.i2c.light.opt4060.publish_hz">(),
    param_cget<"role.i2c.light.opt4060.scan_hz">(), frequency);
  if (not settings) {
    return DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::INVALID_PARAM,
                        std::to_underlying(settings.error()));
  }
  const float deltaRelPct = param_cget<"role.i2c.light.opt4060.delta_rel_pct">();
  if (not std::isfinite(deltaRelPct) || deltaRelPct < 0.0f || deltaRelPct > 100.0f) {
    return DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::INVALID_PARAM, 4U);
  }
  timing = *settings;
  publication.configure({
    .intervalUs = timing.publishIntervalUs,
    .heartbeatUs = 1000U * static_cast<uint32_t>(
      param_cget<"role.i2c.light.opt4060.heartbeat_ms">()),
    .deltaAbs = static_cast<uint32_t>(param_cget<"role.i2c.light.opt4060.delta_abs">()),
    .deltaRelPct = deltaRelPct,
    .onChange = timing.onChange,
  });
  if (const auto status = I2CPeriph::start(); not status) {
    return status;
  }
  useInterrupt = param_cget<"role.i2c.light.opt4060.use_interrupt">();
  if (useInterrupt && not boardResource.tryAcquire(HWResource::PA08)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                        std::to_underlying(HWResource::PA08));
  }
  const auto fail = [this](DeviceStatus status) {
    free_dma(dma);
    dma = nullptr;
    if (useInterrupt) {
      boardResource.release(HWResource::PA08);
    }
    return status;
  };
  DeviceStatus status(DeviceStatus::OPT4060);
  dma = try_new_dma<DmaBuffers>(DeviceStatus::OPT4060, status);
  if (not status) {
    return fail(status);
  }
  address = static_cast<uint8_t>(param_cget<"role.i2c.light.opt4060.address">());
  sensorId = static_cast<uint8_t>(param_cget<"role.i2c.light.opt4060.sensor_id">());
  if (not initialize()) {
    return fail(DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::NOT_FOUND));
  }
  if (chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(1536U),
        "opt4060", NORMALPRIO, &Trampoline<&Opt4060Role::run>::fn, this) == nullptr) {
    (void) writeRegister(configurationRegister, powerDown);
    return fail(DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::HEAP_FULL));
  }
  node.infoCb("OPT4060: addr=0x%02x, %s, scan=%lu us, conversion=%lu us/channel",
              address, timing.onChange ? "on-change" : "periodic",
              static_cast<unsigned long>(timing.scanIntervalUs),
              static_cast<unsigned long>(timing.conversionUs));
  return DeviceStatus(DeviceStatus::OPT4060);
}

msg_t Opt4060Role::transfer(size_t txLength, size_t rxLength)
{
  i2cAcquireBus(&ExternalI2CD);
  const msg_t result = i2cMasterTransmitTimeout(&ExternalI2CD, address,
    dma->tx, txLength, dma->rx, rxLength, TIME_MS2I(20U));
  if (result != MSG_OK) {
    const i2cflags_t errors = i2cGetErrors(&ExternalI2CD);
    // An absent address during probing is not a stuck bus.
    if ((result == MSG_TIMEOUT) ||
        ((errors & ~static_cast<i2cflags_t>(I2C_ACK_FAILURE)) != I2C_NO_ERROR)) {
      I2CPeriph::resetLocked();
    }
  }
  i2cReleaseBus(&ExternalI2CD);
  return result;
}

bool Opt4060Role::readRegister(uint8_t reg, uint16_t& value)
{
  dma->tx[0] = reg;
  if (transfer(1U, 2U) != MSG_OK) {
    return false;
  }
  value = Opt4060Sample::readBigEndian16(dma->rx);
  return true;
}

bool Opt4060Role::writeRegister(uint8_t reg, uint16_t value)
{
  dma->tx[0] = reg;
  dma->tx[1] = static_cast<uint8_t>(value >> 8U);
  dma->tx[2] = static_cast<uint8_t>(value);
  return transfer(3U, 0U) == MSG_OK;
}

bool Opt4060Role::initialize()
{
  haveFrame = false;
  overloaded = false;
  const uint8_t preferred = (address >= 0x44U && address <= 0x47U) ? address : 0x44U;
  for (uint8_t offset = 0U; offset < 4U; ++offset) {
    address = static_cast<uint8_t>(0x44U + ((preferred - 0x44U + offset) % 4U));
    uint16_t id = 0U;
    if (readRegister(idRegister, id) && ((id & 0x3FFFU) == 0x0821U)) {
      if (not writeRegister(configurationRegister, powerDown) ||
          not writeRegister(configuration2Register, configuration2) ||
          not writeRegister(configurationRegister, timing.triggerConfig() & ~modeMask)) {
        return false;
      }
      chThdSleepMilliseconds(2U);
      return true;
    }
  }
  return false;
}

Opt4060Sample::Result Opt4060Role::readSample()
{
  using Result = Opt4060Sample::Result;
  dma->tx[0] = resultRegister;
  if (transfer(1U, sizeof(dma->rx)) != MSG_OK) {
    return Result::Corrupt;
  }
  Opt4060Sample::Frame next;
  const Result result = Opt4060Sample::decodeFrame(
    dma->rx, next, haveFrame ? &frame : nullptr);
  if (result != Result::Valid) {
    return result;
  }
  uint16_t status = 0U;
  if (not readRegister(statusRegister, status)) {
    return Result::Corrupt;
  }
  overloaded = (status & (1U << 3U)) != 0U;
  frame = next;
  haveFrame = true;
  return Result::Valid;
}

void Opt4060Role::observe(bool decoded, uint64_t timestampUs)
{
  microcan_light_Measurement message = {};
  message.timestamp_us = timestampUs;
  message.conversion_time_us = timing.conversionUs;
  message.sensor_id = sensorId;
  message.status = MICROCAN_LIGHT_MEASUREMENT_STATUS_ERROR;
  if (decoded) {
    std::ranges::copy(frame.counts, message.rgbw);
    message.status = overloaded ? MICROCAN_LIGHT_MEASUREMENT_STATUS_SATURATED
                                : MICROCAN_LIGHT_MEASUREMENT_STATUS_VALID;
  }
  publication.observe(message);
}

void Opt4060Role::run(void *)
{
  // Change the pad only after successful startup and exclusive reservation.
  if (useInterrupt) {
    palSetLineMode(interruptLine, PAL_MODE_INPUT_PULLUP);
    palEnableLineEvent(interruptLine, PAL_EVENT_MODE_FALLING_EDGE);
  }
  bool available = true;
  bool acquiring = false;
  bool readySignal = false;
  unsigned failures = 0U;
  uint64_t nextScan = nowUs();
  uint64_t nextRead = 0U;
  uint64_t conversionDeadline = 0U;
  uint64_t nextRetry = 0U;
  const auto finish = [&](bool decoded) {
    acquiring = false;
    if (decoded) {
      failures = 0U;
    } else {
      // Stop a timed-out conversion before starting another one.
      (void) writeRegister(configurationRegister, powerDown);
      if (++failures >= 3U) {
        available = false;
        nextRetry = nowUs() + 5'000'000U;
      }
    }
    observe(decoded, nowUs());
  };

  while (true) {
    uint64_t now = nowUs();
    if (acquiring && (readySignal || now >= nextRead)) {
      uint16_t configuration = 0U;
      if (not readRegister(configurationRegister, configuration)) {
        finish(false);
      } else if ((configuration & ~modeMask) != (timing.triggerConfig() & ~modeMask)) {
        // A sensor reset must not turn reset-valued result registers into a
        // valid dark reading, or label a different exposure with our settings.
        finish(false);
      } else if ((configuration & modeMask) == 0U) {
        // One-shot M bits reset only after the full cycle. Never decode an
        // old/partly updated frame merely because a wait timed out.
        finish(readSample() == Opt4060Sample::Result::Valid);
      } else if (nowUs() >= conversionDeadline) {
        finish(false);
      } else {
        nextRead = std::min(conversionDeadline, nowUs() + 500U);
      }
    }
    readySignal = false;
    now = nowUs();
    if (not available && now >= nextRetry) {
      available = initialize();
      failures = 0U;
      nextRetry = nowUs() + 5'000'000U;
      nextScan = nowUs();
    }
    if (const auto *pending = publication.due(nowUs())) {
      auto message = *pending;
      const bool queued = m_node->sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW)
                          == UAVCAN::Node::CAN_OK;
      publication.complete(nowUs(), queued);
    }
    now = nowUs();
    if (not acquiring && now >= nextScan) {
      nextScan = now + timing.scanIntervalUs;
      if (not available) {
        observe(false, now);
      } else if (writeRegister(configurationRegister, timing.triggerConfig())) {
        acquiring = true;
        // Anchor after the trigger transaction, which may have waited for the
        // shared bus. Never catch up by starting two shots too close together.
        const uint64_t triggered = nowUs();
        nextScan = triggered + timing.scanIntervalUs;
        const uint64_t nominalEnd = triggered + 4U * timing.conversionUs;
        nextRead = nominalEnd + 500U;
        conversionDeadline = nominalEnd + 2000U;
      } else {
        finish(false);
        nextScan = nowUs() + timing.scanIntervalUs;
      }
    }

    uint64_t wake = acquiring ? nextRead : nextScan;
    if (not available) {
      wake = std::min(wake, nextRetry);
    }
    if (const auto deadline = publication.deadline()) {
      wake = std::min(wake, *deadline);
    }
    now = nowUs();
    if (wake > now) {
      const auto delay = TIME_US2I(static_cast<uint32_t>(wake - now));
      if (useInterrupt && acquiring) {
        readySignal = palWaitLineTimeout(interruptLine, delay) == MSG_OK;
      } else {
        chThdSleep(delay);
      }
    }
  }
}
#endif
