/**
 * @file servoRole.hpp
 * @brief Role that routes actuator commands to servo backends.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

/**
 * @brief Role handling uavcan.equipment.actuator.ArrayCommand messages.
 */
class ServoRole final : public RoleBase, public RoleCrtp<ServoRole> {
public:
  /** @brief Subscribe to actuator command messages. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Initialize servo backends based on parameters. */
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  /** @brief Dispatch incoming actuator array commands. */
  static void processActuatorArrayCommand(CanardRxTransfer *,
				   const  uavcan_equipment_actuator_ArrayCommand &msg);
  static bool m_role_servo_pwm;
  static bool m_role_servo_smart;
};
