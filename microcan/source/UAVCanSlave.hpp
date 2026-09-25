/**
 * @file UAVCanSlave.hpp
 * @brief UAVCAN node wrapper used by the application.
 */
#pragma once
#include "roleStatus.hpp"
#include "UAVCAN/pubSub.hpp"

namespace CANSlave {
  /**
   * @brief Start UAVCAN, the enabled shell and, unless identifying, other roles.
   * @param identificationMode Keep management and the configured shell in MAINTENANCE mode.
   */
  DeviceStatus  start(int8_t nodeId, bool dynamicId_fd, bool identificationMode);
  /** @brief Return the configured node ID. */
  uint8_t	getNodeId();
  /** @brief Return the singleton UAVCAN node instance. */
  UAVCAN::Node& getInstance();
}
