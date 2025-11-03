#pragma once
#include "roleStatus.hpp"

namespace CANSlave {
  DeviceStatus  start(int8_t nodeId);
  uint8_t	getNodeId();
}
