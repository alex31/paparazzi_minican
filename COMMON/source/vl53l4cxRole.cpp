/** @file VL53L4CX acquisition and I2C port extracted from imav2026. */
#include "roleConf.h"
#if USE_VL53L4CX_ROLE

#include "vl53l4cxRole.hpp"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"
#include "vl53l4cx.h"
#include <uavcan.equipment.range_sensor.Measurement.h>
#include <algorithm>
#include <cstring>
#include <limits>

struct Vl53l4cxState {
  struct DmaBuffers {
    uint8_t tx[260];
    uint8_t rx[260];
  };
  VL53L4CX_Object_t device = {};
  DmaBuffers *dma = nullptr;
  uint8_t pendingRegister[2] = {};
  uint16_t pendingAddress = 0U;
  bool pendingRead = false;
};

extern "C" uint8_t *vl53l4cx_work_buffer(const void *device, uint32_t requiredSize)
{
  const auto *role = Vl53l4cxRole::singleton;
  if (role == nullptr || role->sensor == nullptr || role->sensor->dma == nullptr ||
      device != &role->sensor->device || requiredSize > sizeof(role->sensor->dma->tx)) {
    return nullptr;
  }
  return role->sensor->dma->tx;
}

extern "C" void vl53l4cx_sleep_us(uint32_t microseconds)
{
  if (microseconds > 0U) {
    chThdSleepMicroseconds(microseconds);
  }
}

DeviceStatus Vl53l4cxRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  return DeviceStatus(DeviceStatus::VL53L4CX);
}

DeviceStatus Vl53l4cxRole::start(UAVCAN::Node& node)
{
  m_node = &node;
  const uint32_t frequency = param_cget<"bus.i2c.frequency_khz">();
  if (frequency < 100U || frequency > 400U) {
    return DeviceStatus(DeviceStatus::VL53L4CX, DeviceStatus::I2C_FREQ_INVALID,
                        static_cast<uint16_t>(frequency));
  }
  if (const auto status = I2CPeriph::start(); not status) {
    return status;
  }
  void *memory = malloc_m(sizeof(Vl53l4cxState));
  if (memory == nullptr) {
    return DeviceStatus(DeviceStatus::VL53L4CX, DeviceStatus::HEAP_FULL);
  }
  sensor = new (memory) Vl53l4cxState;
  const auto fail = [this](DeviceStatus status) {
    free_dma(sensor->dma);
    sensor->~Vl53l4cxState();
    free_m(sensor);
    sensor = nullptr;
    return status;
  };
  DeviceStatus status(DeviceStatus::VL53L4CX);
  sensor->dma = try_new_dma<Vl53l4cxState::DmaBuffers>(DeviceStatus::VL53L4CX, status);
  if (not status) {
    return fail(status);
  }
  periodMs = static_cast<uint32_t>(param_cget<"role.i2c.range.vl53l4cx.period_ms">());
  sensorId = static_cast<uint8_t>(param_cget<"role.i2c.range.vl53l4cx.sensor_id">());
  if (not initialize()) {
    return fail(DeviceStatus(DeviceStatus::VL53L4CX, DeviceStatus::NOT_FOUND));
  }
  if (chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(2048U),
        "vl53l4cx", NORMALPRIO, &Trampoline<&Vl53l4cxRole::run>::fn, this) == nullptr) {
    (void) VL53L4CX_DeInit(&sensor->device);
    return fail(DeviceStatus(DeviceStatus::VL53L4CX, DeviceStatus::HEAP_FULL));
  }
  return DeviceStatus(DeviceStatus::VL53L4CX);
}

int32_t Vl53l4cxRole::busInit()
{
  return singleton != nullptr && singleton->sensor != nullptr ? 0 : -1;
}

int32_t Vl53l4cxRole::busDeinit() { return 0; }

int32_t Vl53l4cxRole::getTick()
{
  return static_cast<int32_t>(TIME_I2MS(chVTGetSystemTimeX()));
}

int32_t Vl53l4cxRole::busWrite(uint16_t address, uint8_t *data, uint16_t length)
{
  if (singleton == nullptr || singleton->sensor == nullptr || data == nullptr) {
    return -1;
  }
  auto& state = *singleton->sensor;
  // ST splits register selection and reading. Defer selection to combine them
  // into one I2C transaction with a repeated START under the shared mutex.
  if (length == 2U) {
    std::memcpy(state.pendingRegister, data, 2U);
    state.pendingAddress = address;
    state.pendingRead = true;
    return 0;
  }
  state.pendingRead = false;
  return singleton->transfer(address, data, length, 0U) == MSG_OK ? 0 : -1;
}

int32_t Vl53l4cxRole::busRead(uint16_t address, uint8_t *data, uint16_t length)
{
  if (singleton == nullptr || singleton->sensor == nullptr || data == nullptr) {
    return -1;
  }
  auto& state = *singleton->sensor;
  if (not state.pendingRead || state.pendingAddress != address) {
    return -1;
  }
  state.pendingRead = false;
  if (singleton->transfer(address, state.pendingRegister, 2U, length) != MSG_OK) {
    return -1;
  }
  std::memcpy(data, state.dma->rx, length);
  return 0;
}

msg_t Vl53l4cxRole::transfer(uint16_t address, const uint8_t *tx,
                             size_t txLength, size_t rxLength)
{
  auto& dma = *sensor->dma;
  if (txLength > sizeof(dma.tx) || rxLength > sizeof(dma.rx) || address > 0xFEU) {
    return MSG_RESET;
  }
  if (txLength != 0U && tx != dma.tx) {
    std::memcpy(dma.tx, tx, txLength);
  }
  i2cAcquireBus(&ExternalI2CD);
  const msg_t result = i2cMasterTransmitTimeout(&ExternalI2CD,
    static_cast<i2caddr_t>(address >> 1U), dma.tx, txLength, dma.rx, rxLength,
    TIME_MS2I(50U));
  if (result != MSG_OK) {
    const i2cflags_t errors = i2cGetErrors(&ExternalI2CD);
    if (result == MSG_TIMEOUT ||
        (errors & ~static_cast<i2cflags_t>(I2C_ACK_FAILURE)) != I2C_NO_ERROR) {
      I2CPeriph::resetLocked();
    }
  }
  i2cReleaseBus(&ExternalI2CD);
  return result;
}

bool Vl53l4cxRole::initialize()
{
  auto& device = sensor->device;
  // Stop a previously active measurement before rebuilding the ST context.
  if (device.IsRanging != 0U) {
    (void) VL53L4CX_Stop(&device);
  }
  std::memset(&device, 0, sizeof(device));
  sensor->pendingRead = false;
  VL53L4CX_IO_t io = {
    .Init = &Vl53l4cxRole::busInit,
    .DeInit = &Vl53l4cxRole::busDeinit,
    .Address = VL53L4CX_DEVICE_ADDRESS,
    .WriteReg = &Vl53l4cxRole::busWrite,
    .ReadReg = &Vl53l4cxRole::busRead,
    .GetTick = &Vl53l4cxRole::getTick,
  };
  uint32_t id = 0U;
  if (VL53L4CX_RegisterBusIO(&device, &io) != VL53L4CX_OK ||
      VL53L4CX_ReadID(&device, &id) != VL53L4CX_OK || id != VL53L4CX_ID ||
      VL53L4CX_Init(&device) != VL53L4CX_OK) {
    return false;
  }
  VL53L4CX_ProfileConfig_t profile = {
    .RangingProfile = VL53L4CX_PROFILE_LONG,
    .TimingBudget = 30U,
    .Frequency = 0U,
    .EnableAmbient = 1U,
    .EnableSignal = 1U,
  };
  return VL53L4CX_ConfigProfile(&device, &profile) == VL53L4CX_OK;
}

bool Vl53l4cxRole::measure()
{
  auto& device = sensor->device;
  VL53L4CX_Result_t result = {};
  int32_t status = VL53L4CX_Start(&device, VL53L4CX_MODE_ASYNC_ONESHOT);
  if (status == VL53L4CX_OK) {
    const systime_t start = chVTGetSystemTimeX();
    do {
      status = VL53L4CX_GetDistance(&device, &result);
      if (status != VL53L4CX_TIMEOUT) {
        break;
      }
      chThdSleepMilliseconds(1U);
    } while (chTimeDiffX(start, chVTGetSystemTimeX()) < TIME_MS2I(100U));
  }
  if (device.IsRanging != 0U) {
    const int32_t stopStatus = VL53L4CX_Stop(&device);
    if (status == VL53L4CX_OK) {
      status = stopStatus;
    }
  }
  if (status != VL53L4CX_OK) {
    publish(false);
    return false;
  }
  const auto& zone = result.ZoneResult[0];
  const size_t count = std::min(static_cast<size_t>(zone.NumberOfTargets),
                              static_cast<size_t>(VL53L4CX_NB_TARGET_PER_ZONE));
  uint32_t nearest = UINT32_MAX;
  for (size_t target = 0U; target < count; ++target) {
    if (zone.Status[target] == 0U) {
      nearest = std::min(nearest, zone.Distance[target]);
    }
  }
  publish(nearest != UINT32_MAX,
          nearest != UINT32_MAX ? static_cast<float>(nearest) * 0.001f : 0.0f);
  return true; // No valid target is a measurement result, not an I2C failure.
}

void Vl53l4cxRole::publish(bool valid, float rangeMetres)
{
  uavcan_equipment_range_sensor_Measurement message = {};
  // Zero means unknown: this node has no synchronized network timestamp.
  message.timestamp.usec = 0U;
  message.sensor_id = sensorId;
  message.beam_orientation_in_body_frame.orientation_defined = false;
  message.field_of_view = 0.3141592654f;
  message.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
  message.reading_type = valid
    ? UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE
    : UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
  message.range = valid ? rangeMetres : std::numeric_limits<float>::quiet_NaN();
  m_node->sendBroadcast(message, CANARD_TRANSFER_PRIORITY_MEDIUM);
}

void Vl53l4cxRole::run(void *)
{
  bool available = true;
  unsigned failures = 0U;
  systime_t lastRetry = chVTGetSystemTimeX();
  while (true) {
    const systime_t start = chVTGetSystemTimeX();
    if (available) {
      if (measure()) {
        failures = 0U;
      } else if (++failures >= 3U) {
        available = false;
        lastRetry = chVTGetSystemTimeX();
      }
    } else {
      publish(false);
      if (chTimeDiffX(lastRetry, start) >= TIME_MS2I(5000U)) {
        lastRetry = start;
        available = initialize();
        failures = 0U;
      }
    }
    const sysinterval_t elapsed = chTimeDiffX(start, chVTGetSystemTimeX());
    const sysinterval_t period = TIME_MS2I(periodMs);
    // No catch-up burst after a slow initialization or a failed bus transfer.
    chThdSleep(elapsed < period ? period - elapsed : TIME_MS2I(1U));
  }
}
#endif
