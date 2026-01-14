#pragma once
#include "roleStatus.hpp"
#include "UAVCAN/pubSub.hpp"

namespace CANSlave {
  DeviceStatus  start(int8_t nodeId, bool dynamicId_fd);
  uint8_t	getNodeId();
  UAVCAN::Node& getInstance();
}
