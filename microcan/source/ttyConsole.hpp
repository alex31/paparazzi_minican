/** @brief Optional diagnostic shell role on J3 / LPUART1. */
#pragma once
#include "UAVCanSlave.hpp"
#include "roleBase.hpp"

#ifdef TRACE
class ShellRole final : public RoleBase {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;
};
#endif
