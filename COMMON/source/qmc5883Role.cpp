/**
 * @file qmc5883Role.cpp
 * @brief QMC5883L magnetometer role: fixed config, publishes MagneticFieldStrength2.
 */

#include "qmc5883Role.hpp"
#include <algorithm>
#include "stdutil.h"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"
#include "UAVCAN/persistantParam.hpp"

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "QMC5883L driver assumes little-endian MCU");

namespace {
  constexpr uint8_t qmcAddr = 0x0D;
  enum Register : uint8_t {
    REG_DATA = 0x00,
    REG_STATUS = 0x06,
    REG_CONTROL_1 = 0x09,
    REG_CONTROL_2 = 0x0A,
    REG_RESET_PERIOD = 0x0B,
  };

  enum Ctrl1Bits : uint8_t {
    CTRL1_OSR_512 = 0x00,
    CTRL1_OSR_256 = 0x40,
    CTRL1_OSR_128 = 0x80,
    CTRL1_OSR_64  = 0xC0,

    CTRL1_ODR_10HZ  = 0x00,
    CTRL1_ODR_50HZ  = 0x04,
    CTRL1_ODR_100HZ = 0x08,
    CTRL1_ODR_200HZ = 0x0C,

    CTRL1_MODE_STBY = 0x00,
    CTRL1_MODE_CONT = 0x01,
    CTRL1_MODE_SINGLE = 0x02,

    CTRL1_RANGE_2G = 0x00,
    CTRL1_RANGE_8G = 0x10,
  };

  enum Ctrl2Bits : uint8_t {
    CTRL2_SOFT_RESET = 0x80,
    CTRL2_ROL_PNT = 0x40,
    CTRL2_INT_ENB = 0x01,
  };

  enum StatusBits : uint8_t {
    STATUS_DRDY = 0x01,
  };

  constexpr float countsPerGauss2G = 12000.0f;
  constexpr float countsPerGauss8G = 3000.0f;
}


DeviceStatus Qmc5883Role::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::MAG_QMC5883);
}


DeviceStatus Qmc5883Role::start(UAVCAN::Node& node)
{
  m_node = &node;

  const auto i2cStatus = I2CPeriph::start();
  if (!i2cStatus) {
    return i2cStatus;
  }

  const uint16_t range = PARAM_CGET("role.i2c.magnetometer.q5883.range");
  uint8_t rangeBits = CTRL1_RANGE_8G;
  if (range == 2) {
    countsPerGauss = countsPerGauss2G;
    rangeBits = CTRL1_RANGE_2G;
  } else if (range == 8) {
    countsPerGauss = countsPerGauss8G;
    rangeBits = CTRL1_RANGE_8G;
  } else {
    return DeviceStatus(DeviceStatus::MAG_QMC5883, DeviceStatus::INVALID_PARAM);
  }

  rotDeg = PARAM_CGET("role.i2c.magnetometer.q5883.rot_deg");
  if (!((rotDeg == 0) || (rotDeg == 90) || (rotDeg == 180) || (rotDeg == 270))) {
    return DeviceStatus(DeviceStatus::MAG_QMC5883, DeviceStatus::INVALID_PARAM);
  }

  sensorId = static_cast<uint8_t>(PARAM_CGET("role.i2c.magnetometer.q5883.sensor_id"));

  dmaBuf = static_cast<DmaBuffers*>(malloc_dma(sizeof(DmaBuffers)));
  if (!dmaBuf) {
    return DeviceStatus(DeviceStatus::MAG_QMC5883, DeviceStatus::DMA_HEAP_FULL);
  }

  msg_t status = MSG_OK;

  static const uint8_t resetPeriod[] = {REG_RESET_PERIOD, 0x01};
  status = i2cMasterTransmitTimeout(&ExternalI2CD, qmcAddr, resetPeriod, sizeof(resetPeriod),
				    nullptr, 0, TIME_MS2I(10));

  if (status == MSG_OK) {
    static const uint8_t ctrl2[] = {REG_CONTROL_2, CTRL2_SOFT_RESET};
    status = i2cMasterTransmitTimeout(&ExternalI2CD, qmcAddr, ctrl2, sizeof(ctrl2),
				      nullptr, 0, TIME_MS2I(10));
  }

  if (status == MSG_OK) {
    dmaBuf->ctrl1[0] = REG_CONTROL_1;
    dmaBuf->ctrl1[1] = static_cast<uint8_t>(CTRL1_OSR_256 | CTRL1_ODR_50HZ |
					    rangeBits | CTRL1_MODE_CONT);
    status = i2cMasterTransmitTimeout(&ExternalI2CD, qmcAddr, dmaBuf->ctrl1, sizeof(dmaBuf->ctrl1),
				      nullptr, 0, TIME_MS2I(10));
  }

  if (status != MSG_OK) {
    return DeviceStatus(DeviceStatus::MAG_QMC5883, DeviceStatus::I2C_TIMOUT);
  }

  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(768), "qmc5883",
		      NORMALPRIO,
		      &Trampoline<&Qmc5883Role::periodic>::fn,
		      this);
  return DeviceStatus(DeviceStatus::MAG_QMC5883);
}


void Qmc5883Role::periodic(void *)
{
  uavcan_equipment_ahrs_MagneticFieldStrength2 msg = {};
  msg.sensor_id = sensorId;
  msg.magnetic_field_covariance.len = 0;
  static const uint8_t statusReg = REG_STATUS;
  static const uint8_t regData = REG_DATA;

  while (true) {
    dmaBuf->status = 0;
    msg_t ret = i2cMasterTransmitTimeout(&ExternalI2CD, qmcAddr, &statusReg, 1,
					 &dmaBuf->status, 1, TIME_MS2I(10));
    if (ret != MSG_OK) {
      I2CPeriph::reset();
      chThdSleepMilliseconds(10);
      continue;
    }

    if ((dmaBuf->status & STATUS_DRDY) == 0) {
      chThdSleepMilliseconds(5);
      continue;
    }

    // QMC5883L shares endianness with STM32; read axes directly
    ret = i2cMasterTransmitTimeout(&ExternalI2CD, qmcAddr, &regData, 1,
				   reinterpret_cast<uint8_t*>(dmaBuf->axes), 
				   sizeof(dmaBuf->axes),
				   TIME_MS2I(10));
    if (ret != MSG_OK) {
      I2CPeriph::reset();
      chThdSleepMilliseconds(10);
      continue;
    }

    const int16_t rawX = dmaBuf->axes[0];
    const int16_t rawY = dmaBuf->axes[1];
    const int16_t rawZ = dmaBuf->axes[2];

    float mx = static_cast<float>(rawX) / countsPerGauss;
    float my = static_cast<float>(rawY) / countsPerGauss;
    float mz = static_cast<float>(rawZ) / countsPerGauss;

    // Apply discrete yaw rotations to match Paparazzi convention.
    switch (rotDeg) {
    case 90:
      std::swap(mx, my);
      mx = -mx;
      break;
    case 180:
      mx = -mx; my = -my;
      break;
    case 270:
      std::swap(mx, my);
      my = -my;
      break;
    default:
      break;
    }

    msg.magnetic_field_ga[0] = mx;
    msg.magnetic_field_ga[1] = my;
    msg.magnetic_field_ga[2] = mz;

    m_node->sendBroadcast(msg, CANARD_TRANSFER_PRIORITY_LOW);
  }
}
