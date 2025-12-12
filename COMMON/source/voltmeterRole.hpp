#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"

class VoltmeterRole final : public RoleBase, public RoleCrtp<VoltmeterRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;
};
