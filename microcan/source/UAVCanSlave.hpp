/**
 * @file UAVCanSlave.hpp
 * @brief UAVCAN node wrapper used by the application.
 */
#pragma once
#include "roleStatus.hpp"
#include "UAVCAN/pubSub.hpp"

namespace CANSlave {
  /** @brief Initialize the UAVCAN node and start it. */
  DeviceStatus  start(int8_t nodeId, bool dynamicId_fd);
  /** @brief Return the configured node ID. */
  uint8_t	getNodeId();
  /** @brief Return the singleton UAVCAN node instance. */
  UAVCAN::Node& getInstance();
}
