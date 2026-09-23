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
  /** @brief Recover the I2C peripheral while serializing all bus users. */
  void reset();
  /** @brief Recover I2C when the caller already owns ExternalI2CD's mutex. */
  void resetLocked();
};
