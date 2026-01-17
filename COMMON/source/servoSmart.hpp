/**
 * @file servoSmart.hpp
 * @brief API for smart servo setpoint publishing.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "roleStatus.hpp"
#include  "UAVCAN/pubSub.hpp"

namespace ServoSmart {
  /** @brief Initialize the smart servo publisher role. */
  DeviceStatus start(UAVCAN::Node& node);
  /** @brief Publish a PWM setpoint for the given servo index. */
  void setPwm(uint8_t index, float value);
  /** @brief Publish a unitless setpoint for the given servo index. */
  void setUnitless(uint8_t index, float value);
  /** @brief Publish a torque setpoint for the given servo index. */
  void setTorque(uint8_t index, float value);
  /** @brief Publish a speed setpoint for the given servo index. */
  void setSpeed(uint8_t index, float value);
}
