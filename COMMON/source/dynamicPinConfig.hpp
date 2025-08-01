#pragma once

#include <ch.h>
#include <hal.h>


namespace DynPin {
#ifdef  BOARD_ENAC_MICROCANv3
  enum class Scenario {
    UART, I2C, SPI, PWM, DSHOT
  };

  void inactiveAllSharedPins();
  void setScenario(Scenario s);
#endif
  bool isFirmwareMatchHardware();
  bool i2cUnhangBus(I2CDriver *i2cd);
  void i2cActivatePullup();
}
