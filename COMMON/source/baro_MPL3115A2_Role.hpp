/**
 * @file baro_MPL3115A2_Role.hpp
 * @brief Role wrapper for the MPL3115A2 barometer sensor.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

/**
 * @brief Role exposing MPL3115A2 pressure/temperature data over UAVCAN.
 */
class Baro_MPL3115A2_Role final : public RoleBase, public RoleCrtp<Baro_MPL3115A2_Role> {
public:
  /** @brief Subscribe to the UAVCAN topics used by this role. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Initialize the sensor and start the periodic polling task. */
  DeviceStatus start(UAVCAN::Node& node) override;
  /** @brief Read the device ID register. */
  DeviceStatus getDevId(uint8_t *devId);
  /** @brief Read the current pressure in Pascals. */
  DeviceStatus getPressure(float *pressure);
  /** @brief Print the current pressure for debug output. */
  DeviceStatus printPressure();
  /** @brief Read the current temperature in degrees Celsius. */
  DeviceStatus getTemperature(float *temperature);

private:
  /** @brief Periodic polling routine. */
  void periodic();
  /** @brief Recover the I2C bus after an error. */
  void resetI2C();
};
