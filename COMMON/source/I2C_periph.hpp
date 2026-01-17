/**
 * @file I2C_periph.hpp
 * @brief Helper API for initializing and resetting the shared I2C peripheral.
 */
#pragma once

#include "roleStatus.hpp"

namespace I2CPeriph
{
  /** @brief Initialize the shared I2C peripheral. */
  DeviceStatus start();
  /** @brief Reset the I2C peripheral to recover from a bus error. */
  void reset();
};
