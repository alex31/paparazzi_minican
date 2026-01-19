#include "roleConf.h"

#if USE_TEMPLATE_ROLE

#include "templateRole.hpp"
#include <algorithm>
#include "UAVCAN/persistantParam.hpp"

DeviceStatus TemplateRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&TemplateRole::processNodeStatus>::fn>();
  return DeviceStatus(DeviceStatus::TEMPLATE_ROLE);
}

DeviceStatus TemplateRole::start(UAVCAN::Node&)
{
  const uint16_t cfg = param_cget<"role.template.log_every">();
  logEvery = static_cast<uint16_t>(std::clamp<uint32_t>(cfg, 0U, 1000U));
  return DeviceStatus(DeviceStatus::TEMPLATE_ROLE);
}

void TemplateRole::processNodeStatus(CanardRxTransfer* transfer,
                                     const uavcan_protocol_NodeStatus& msg)
{
  ++rxCount;
  lastNodeId = transfer ? transfer->source_node_id : 0;
  lastUptime = msg.uptime_sec;

  if (m_node && (logEvery > 0U) && ((rxCount % logEvery) == 0U)) {
    m_node->infoCb("templateRole: node=%u uptime=%lu", lastNodeId, lastUptime);
  }
}

#endif // USE_TEMPLATE_ROLE
