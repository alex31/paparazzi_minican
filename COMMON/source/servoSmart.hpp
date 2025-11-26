#pragma once

#include <ch.h>
#include <hal.h>
#include "roleStatus.hpp"
#include  "UAVCAN/pubSub.hpp"

namespace ServoSmart {
  DeviceStatus start(UAVCAN::Node& node);
  void setPwm(uint8_t index, float value);
  void setUnitless(uint8_t index, float value);
  void setTorque(uint8_t index, float value);
  void setSpeed(uint8_t index, float value);
}
