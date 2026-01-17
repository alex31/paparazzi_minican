/**
 * @file voltmeterRole.hpp
 * @brief Voltmeter role definition for battery voltage indication.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"

/**
 * @brief Role that samples battery voltage and drives LED indication.
 */
class VoltmeterRole final : public RoleBase, public RoleCrtp<VoltmeterRole> {
public:
  /** @brief Subscribe to UAVCAN messages used by this role. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Start the voltmeter processing thread. */
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  /** @brief Handle incoming GNSS Fix2 updates for GPS-based LED blanking. */
  void processFix2(CanardRxTransfer*, const uavcan_equipment_gnss_Fix2& msg);
  /** @brief Background thread that updates voltage indication. */
  static void voltmeterThread(void* arg);

  float m_groundSpeedMps = 0.0f;
  systime_t m_lastGpsUpdate = 0;
  bool m_gpsSpeedValid = false;
};
