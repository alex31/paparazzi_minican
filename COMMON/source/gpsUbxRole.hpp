#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "etl/span.h"


class GpsUBX final : public RoleBase, public RoleCrtp<GpsUBX> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  static constexpr size_t maxUbxFrameSize = 640U;
  uint8_t frame[maxUbxFrameSize];
  void periodic(void *);

};
