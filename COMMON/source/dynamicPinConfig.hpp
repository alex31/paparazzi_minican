/**
 * @file dynamicPinConfig.hpp
 * @brief Helpers to configure shared pins dynamically on MICROCAN.
 */
#pragma once

#include <ch.h>
#include <hal.h>


namespace DynPin {
#if PLATFORM_MICROCAN
  /** @brief Shared pin configuration scenarios. */
  enum class Scenario {
    UART, I2C, SPI, PWM, DSHOT
  };

  /** @brief Drive all shared pins to the inactive state. */
  void inactiveAllSharedPins();
  /** @brief Configure pins for the given scenario and optional channel mask. */
  void setScenario(Scenario s, uint8_t mask=0);
#endif
  /** @brief Validate that the firmware matches the detected hardware. */
  bool isFirmwareMatchHardware();
  /** @brief Toggle I2C pins to release a stuck bus. */
  bool i2cUnhangBus(I2CDriver *i2cd);
  /** @brief Enable pullups on the I2C bus. */
  void i2cActivatePullup();
}
