/** @file OPT4060 acquisition extracted from imav2026, without flash detection. */
#include "roleConf.h"
#if USE_OPT4060_ROLE

#include "opt4060Role.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "I2C_periph.hpp"
#include "sensorTelemetry.hpp"
#include <limits>

namespace {
  const ioline_t interruptLine = PAL_LINE(GPIOA, 8U);
  constexpr uint8_t resultRegister = 0x00U;
  constexpr uint8_t configurationRegister = 0x0AU;
  constexpr uint8_t configuration2Register = 0x0BU;
  constexpr uint8_t statusRegister = 0x0CU;
  constexpr uint8_t idRegister = 0x11U;
  // Auto-range, 1.8 ms/channel. INT pulses after all four channels complete.
  constexpr uint16_t powerDown = 0x3088U;
  constexpr uint16_t continuous = 0x30B8U;
  constexpr uint16_t configuration2 = 0x801DU;
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
  if ((frequency < 100U) || (frequency > 400U)) {
    return DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::I2C_FREQ_INVALID,
                        static_cast<uint16_t>(frequency));
  }
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
  publishPeriod = TIME_MS2I(static_cast<uint32_t>(
    param_cget<"role.i2c.light.opt4060.period_ms">()));
  if (not initialize()) {
    return fail(DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::NOT_FOUND));
  }
  if (chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(1536U),
        "opt4060", NORMALPRIO, &Trampoline<&Opt4060Role::run>::fn, this) == nullptr) {
    (void) writeRegister(configurationRegister, powerDown);
    return fail(DeviceStatus(DeviceStatus::OPT4060, DeviceStatus::HEAP_FULL));
  }
  node.infoCb("OPT4060 started: addr=0x%02x, PA8 interrupt=%u", address, useInterrupt);
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
          not writeRegister(configurationRegister, continuous)) {
        return false;
      }
      chThdSleepMilliseconds(8U);
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
  if (next.checkOverload && not readRegister(statusRegister, status)) {
    return Result::Corrupt;
  }
  overloaded = (status & (1U << 3U)) != 0U;
  frame = next;
  haveFrame = true;
  return Result::Valid;
}

void Opt4060Role::publish(bool valid)
{
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  publishSensorValue(*m_node, "opt.ok", valid ? 1.0f : 0.0f);
  constexpr const char *keys[] = {"opt.red", "opt.green", "opt.blue", "opt.clear"};
  for (size_t channel = 0U; channel < frame.counts.size(); ++channel) {
    publishSensorValue(*m_node, keys[channel],
      valid ? static_cast<float>(frame.counts[channel]) : invalid);
  }
  publishSensorValue(*m_node, "opt.ovf", overloaded ? 1.0f : 0.0f);
}

void Opt4060Role::run(void *)
{
  // Change the pad only after successful startup and exclusive reservation.
  if (useInterrupt) {
    palSetLineMode(interruptLine, PAL_MODE_INPUT_PULLUP);
    palEnableLineEvent(interruptLine, PAL_EVENT_MODE_FALLING_EDGE);
  }
  bool available = true;
  bool valid = false;
  unsigned failures = 0U;
  systime_t lastPublish = chVTGetSystemTimeX();
  systime_t lastSample = lastPublish;
  systime_t lastRetry = lastPublish;
  while (true) {
    if (available) {
      if (useInterrupt) {
        (void) palWaitLineTimeout(interruptLine, TIME_MS2I(25U));
      } else {
        chThdSleepMilliseconds(10U);
      }
      const auto result = readSample();
      if (result == Opt4060Sample::Result::Valid) {
        valid = not overloaded;
        failures = 0U;
        lastSample = chVTGetSystemTimeX();
      } else if (result == Opt4060Sample::Result::Corrupt) {
        valid = false;
        ++failures;
      }
    } else {
      chThdSleepMilliseconds(10U);
    }
    const systime_t now = chVTGetSystemTimeX();
    if (available && (failures >= 3U || chTimeDiffX(lastSample, now) >= TIME_MS2I(100U))) {
      available = false;
      valid = false;
      lastRetry = now;
    }
    if (not available && chTimeDiffX(lastRetry, now) >= TIME_MS2I(5000U)) {
      lastRetry = now;
      available = initialize();
      failures = 0U;
      lastSample = chVTGetSystemTimeX();
    }
    if (chTimeDiffX(lastPublish, now) >= publishPeriod) {
      lastPublish = now;
      publish(valid && chTimeDiffX(lastSample, now) < TIME_MS2I(100U));
    }
  }
}
#endif
