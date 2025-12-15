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

private:
  void processFix2(CanardRxTransfer*, const uavcan_equipment_gnss_Fix2& msg);
  static void voltmeterThread(void* arg);

  float m_groundSpeedMps = 0.0f;
  systime_t m_lastGpsUpdate = 0;
  bool m_gpsSpeedValid = false;
};
