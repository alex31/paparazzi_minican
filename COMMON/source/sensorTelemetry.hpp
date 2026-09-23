#pragma once

#include "UAVCAN/pubSub.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"
#include <uavcan.protocol.debug.KeyValue.h>

/** @brief Publish sensor metrics which have no standard DroneCAN message. */
inline void publishSensorValue(UAVCAN::Node& node, const char *key, float value)
{
  uavcan_protocol_debug_KeyValue message = {};
  UAVCAN::dsdlAssign(message.key, key);
  message.value = value;
  node.sendBroadcast(message, CANARD_TRANSFER_PRIORITY_LOW);
}
