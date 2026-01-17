/**
 * @file UAVCanHelper.hpp
 * @brief Utility helpers for UAVCAN logging.
 */
#pragma once
#include "UAVCAN/pubSub.hpp"
#include "etl/string.h"
/*

#define UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG 0
#define UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO 1
#define UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING 2
#define UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR 3
struct uavcan_protocol_debug_LogLevel {
    uint8_t value;
};


struct uavcan_protocol_debug_LogMessage {
    struct uavcan_protocol_debug_LogLevel level;
    struct { uint8_t len; uint8_t data[31]; }source;
    struct { uint8_t len; uint8_t data[90]; }text;
};

 */
namespace UAVCAN::Helper {
  /** @brief Publish a uavcan.protocol.debug.LogMessage. */
  UAVCAN::Node::canStatus_t log(UAVCAN::Node& node, uint8_t level,
				etl::string_view source, etl::string_view text);
}
