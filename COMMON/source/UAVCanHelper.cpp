#include "UAVCanHelper.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"

UAVCAN::Node::canStatus_t UAVCAN::Helper::log(UAVCAN::Node& node, uint8_t level,
					      etl::string_view source, etl::string_view text)
{
  uavcan_protocol_debug_LogMessage lm;
  lm.level.value = level;
  UAVCAN::dsdlAssign(lm.text, text);
  UAVCAN::dsdlAssign(lm.source, source);
  node.infoCb("log level %u from %.*s : [%.*s]", level,
	      lm.source.len, lm.source.data,
	      lm.text.len, lm.text.data);
	      
  return node.sendBroadcast(lm);
}
