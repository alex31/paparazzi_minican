/**
 * @file vl53lxPlatform.c
 * @brief MicroCAN porting layer for the official ST VL53L4CX driver.
 *
 * The ST component algorithms remain unchanged. This application-owned
 * porting layer replaces its global 256-byte I2C scratch buffer with memory
 * supplied by the dynamically allocated VL53L4CX role.
 */

#include "vl53l4cxPort.h"

#include "vl53lx_platform.h"

#include <stddef.h>
#include <string.h>

enum {
  VL53LX_REGISTER_ADDRESS_SIZE = 2U,
};

static uint8_t *work_buffer(VL53LX_DEV dev, uint32_t required_size)
{
  return vl53l4cx_work_buffer((const void *)dev, required_size);
}

static int32_t i2c_write(VL53LX_DEV dev, uint8_t *data, uint32_t count)
{
  if ((dev == NULL) || (dev->IO.WriteReg == NULL) ||
      (count > UINT16_MAX)) {
    return -1;
  }
  return dev->IO.WriteReg(dev->IO.Address, data, (uint16_t)count);
}

static int32_t i2c_read(VL53LX_DEV dev, uint8_t *data, uint32_t count)
{
  if ((dev == NULL) || (dev->IO.ReadReg == NULL) ||
      (count > UINT16_MAX)) {
    return -1;
  }
  return dev->IO.ReadReg(dev->IO.Address, data, (uint16_t)count);
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV dev, uint16_t index,
                               uint8_t *data, uint32_t count)
{
  if ((data == NULL) && (count != 0U)) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }
  if (count > UINT32_MAX - VL53LX_REGISTER_ADDRESS_SIZE) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  const uint32_t transfer_size = count + VL53LX_REGISTER_ADDRESS_SIZE;
  uint8_t *const buffer = work_buffer(dev, transfer_size);
  if (buffer == NULL) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  buffer[0] = (uint8_t)(index >> 8U);
  buffer[1] = (uint8_t)index;
  if (count != 0U) {
    memcpy(&buffer[VL53LX_REGISTER_ADDRESS_SIZE], data, count);
  }

  return i2c_write(dev, buffer, transfer_size) == 0
    ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV dev, uint16_t index,
                              uint8_t *data, uint32_t count)
{
  if ((data == NULL) || (count == 0U)) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  const uint32_t required_size = count > VL53LX_REGISTER_ADDRESS_SIZE
    ? count : VL53LX_REGISTER_ADDRESS_SIZE;
  uint8_t *const buffer = work_buffer(dev, required_size);
  if (buffer == NULL) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  buffer[0] = (uint8_t)(index >> 8U);
  buffer[1] = (uint8_t)index;
  if (i2c_write(dev, buffer, VL53LX_REGISTER_ADDRESS_SIZE) != 0) {
    return VL53LX_ERROR_CONTROL_INTERFACE;
  }
  return i2c_read(dev, data, count) == 0
    ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_DEV dev, uint16_t index, uint8_t data)
{
  return VL53LX_WriteMulti(dev, index, &data, sizeof(data));
}

VL53LX_Error VL53LX_WrWord(VL53LX_DEV dev, uint16_t index, uint16_t data)
{
  uint8_t bytes[2] = {
    (uint8_t)(data >> 8U),
    (uint8_t)data,
  };
  return VL53LX_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV dev, uint16_t index, uint32_t data)
{
  uint8_t bytes[4] = {
    (uint8_t)(data >> 24U),
    (uint8_t)(data >> 16U),
    (uint8_t)(data >> 8U),
    (uint8_t)data,
  };
  return VL53LX_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV dev, uint16_t index,
                               uint8_t and_data, uint8_t or_data)
{
  uint8_t data = 0U;
  VL53LX_Error status = VL53LX_RdByte(dev, index, &data);
  if (status == VL53LX_ERROR_NONE) {
    data = (uint8_t)((data & and_data) | or_data);
    status = VL53LX_WrByte(dev, index, data);
  }
  return status;
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV dev, uint16_t index, uint8_t *data)
{
  return VL53LX_ReadMulti(dev, index, data, sizeof(*data));
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV dev, uint16_t index, uint16_t *data)
{
  if (data == NULL) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  uint8_t bytes[2] = {};
  const VL53LX_Error status =
    VL53LX_ReadMulti(dev, index, bytes, sizeof(bytes));
  if (status == VL53LX_ERROR_NONE) {
    *data = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
  }
  return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_DEV dev, uint16_t index, uint32_t *data)
{
  if (data == NULL) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }

  uint8_t bytes[4] = {};
  const VL53LX_Error status =
    VL53LX_ReadMulti(dev, index, bytes, sizeof(bytes));
  if (status == VL53LX_ERROR_NONE) {
    *data = ((uint32_t)bytes[0] << 24U) |
            ((uint32_t)bytes[1] << 16U) |
            ((uint32_t)bytes[2] << 8U) |
            bytes[3];
  }
  return status;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV dev,
                                 uint32_t *tick_count_ms)
{
  if ((dev == NULL) || (dev->IO.GetTick == NULL) ||
      (tick_count_ms == NULL)) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }
  *tick_count_ms = (uint32_t)dev->IO.GetTick();
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *timer_frequency_hz)
{
  if (timer_frequency_hz == NULL) {
    return VL53LX_ERROR_INVALID_PARAMS;
  }
  *timer_frequency_hz = 0;
  return VL53LX_ERROR_NONE;
}

static VL53LX_Error delay_ms(VL53LX_DEV dev, uint32_t delay)
{
  (void)dev;
  // Yield to the other sensor roles; vendor delays never hold the I2C mutex.
  while (delay > 0U) {
    const uint32_t chunk = delay > 1000U ? 1000U : delay;
    vl53l4cx_sleep_us(chunk * 1000U);
    delay -= chunk;
  }
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_DEV dev, int32_t wait_ms)
{
  return wait_ms > 0 ? delay_ms(dev, (uint32_t)wait_ms)
                     : VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_DEV dev, int32_t wait_us)
{
  (void)dev;
  if (wait_us > 0) {
    vl53l4cx_sleep_us((uint32_t)wait_us);
  }
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_DEV dev, uint32_t timeout_ms,
                                    uint16_t index, uint8_t value,
                                    uint8_t mask, uint32_t poll_delay_ms)
{
  uint32_t start = 0U;
  VL53LX_Error status = VL53LX_GetTickCount(dev, &start);
  if (status != VL53LX_ERROR_NONE) {
    return status;
  }

  uint32_t elapsed = 0U;
  while ((status == VL53LX_ERROR_NONE) && (elapsed < timeout_ms)) {
    uint8_t current_value = 0U;
    status = VL53LX_RdByte(dev, index, &current_value);
    if ((status == VL53LX_ERROR_NONE) &&
        ((current_value & mask) == value)) {
      return VL53LX_ERROR_NONE;
    }
    if ((status == VL53LX_ERROR_NONE) && (poll_delay_ms != 0U)) {
      status = delay_ms(dev, poll_delay_ms);
    }

    uint32_t now = start;
    if (status == VL53LX_ERROR_NONE) {
      status = VL53LX_GetTickCount(dev, &now);
      elapsed = now - start;
    }
  }

  return status == VL53LX_ERROR_NONE ? VL53LX_ERROR_TIME_OUT : status;
}
