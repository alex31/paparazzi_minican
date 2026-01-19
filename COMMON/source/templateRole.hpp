#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

/**
 * @brief Minimal skeleton role for new UAVCAN features.
 *
 * This role is disabled by default (USE_TEMPLATE_ROLE=false) and subscribes to
 * uavcan.protocol.NodeStatus as an example. Copy this file and adapt the
 * message type, resources, and parameters for a new role.
 */
class TemplateRole final : public RoleBase, public RoleCrtp<TemplateRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void processNodeStatus(CanardRxTransfer* transfer,
                         const uavcan_protocol_NodeStatus& msg);

  uint32_t rxCount = 0;
  uint16_t logEvery = 0;
  uint8_t lastNodeId = 0;
  uint32_t lastUptime = 0;
};
