#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

class ServoRole final : public RoleBase, public RoleCrtp<ServoRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  static void processActuatorArrayCommand(CanardRxTransfer *,
				   const  uavcan_equipment_actuator_ArrayCommand &msg);
  static bool m_role_servo_pwm;
  static bool m_role_servo_smart;
};
