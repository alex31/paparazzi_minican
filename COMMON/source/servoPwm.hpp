/**
 * @file servoPwm.hpp
 * @brief PWM servo output API.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"

namespace ServoPWM {
  /** @brief Initialize PWM timers and GPIO for servo output. */
  DeviceStatus start();
  /** @brief Set a PWM microsecond value for the given servo index. */
  void setPwm(uint8_t index, float value);
  /** @brief Set a unitless servo value mapped to PWM timing. */
  void setUnitless(uint8_t index, float value);
}
