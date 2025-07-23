#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"

namespace ServoPWM {
  DeviceStatus start();
  void setPwm(uint8_t index, float value);
  void setUnitless(uint8_t index, float value);
}
