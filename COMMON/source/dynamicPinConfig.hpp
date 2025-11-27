#pragma once

#include <ch.h>
#include <hal.h>


namespace DynPin {
#if PLATFORM_MICROCAN
  enum class Scenario {
    UART, I2C, SPI, PWM, DSHOT
  };

  void inactiveAllSharedPins();
  void setScenario(Scenario s, uint8_t mask=0);
#endif
  bool isFirmwareMatchHardware();
  bool i2cUnhangBus(I2CDriver *i2cd);
  void i2cActivatePullup();
}
