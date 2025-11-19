#pragma once
#include "roleStatus.hpp"

namespace CANSlave {
  DeviceStatus  start(int8_t nodeId, bool dynamicId_fd);
  uint8_t	getNodeId();
}
