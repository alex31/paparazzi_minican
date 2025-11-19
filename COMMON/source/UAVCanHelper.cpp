#include "UAVCanHelper.hpp"

UAVCAN::Node::canStatus_t UAVCAN::Helper::log(UAVCAN::Node& node, uint8_t level,
					      etl::string_view source, etl::string_view text)
{
  uavcan_protocol_debug_LogMessage lm;
  lm.level.value = level;
  lm.text.len = std::min<std::size_t>(text.size(), std::size(lm.text.data));
  std::copy_n(text.data(),  lm.text.len, lm.text.data);

  lm.source.len = std::min<std::size_t>(source.size(), std::size(lm.source.data));
  std::copy_n(source.data(), lm.source.len, lm.source.data);
  node.infoCb("log level %u from %.*s : [%.*s]", level,
	      lm.source.len, lm.source.data,
	      lm.text.len, lm.text.data);
	      
  return node.sendBroadcast(lm);
}
