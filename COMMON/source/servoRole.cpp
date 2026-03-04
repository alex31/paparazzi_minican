/**
 * @file servoRole.cpp
 * @brief Actuator command routing for servo backends.
 */
#include "roleConf.h"

#if USE_SERVO_ROLE

#include "servoRole.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "servoPwm.hpp"
#include "servoSmart.hpp"
#include <bitset>

/** @brief Dispatch actuator commands to PWM and smart servo backends. */
void ServoRole::processActuatorArrayCommand(CanardRxTransfer *,
					    const  uavcan_equipment_actuator_ArrayCommand &msg)
{
  std::bitset<256U> smartImmediateStatusPending;

  for (size_t idx = 0; idx < msg.commands.len; ++idx) {
    const auto &cmd = msg.commands.data[idx];

    if (ServoRole::m_role_servo_pwm) {
      if (cmd.command_type ==
		  UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM) {
	ServoPWM::setPwm(cmd.actuator_id, cmd.command_value);
	      } else if (cmd.command_type ==
			 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
	ServoPWM::setUnitless(cmd.actuator_id, cmd.command_value);
	      }
	    }
	      
    if (ServoRole::m_role_servo_smart) {
      bool smartCommandHandled = false;
      if (cmd.command_type ==
		  UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
	ServoSmart::setUnitless(cmd.actuator_id, cmd.command_value);
	smartCommandHandled = true;
	      } else if (cmd.command_type ==
			 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_FORCE) {
	ServoSmart::setTorque(cmd.actuator_id, cmd.command_value);
	smartCommandHandled = true;
	      } else if (cmd.command_type ==
			 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_SPEED) {
	ServoSmart::setSpeed(cmd.actuator_id, cmd.command_value);
	smartCommandHandled = true;
      }
      if (smartCommandHandled) {
	if (smartImmediateStatusPending[cmd.actuator_id]) {
	  DebugTrace("smart-servo immediate status coalesced id=%u", cmd.actuator_id);
	}
	smartImmediateStatusPending[cmd.actuator_id] = true;
      }
    }
  }

  if (ServoRole::m_role_servo_smart) {
    for (size_t idx = 0; idx < msg.commands.len; ++idx) {
      const uint8_t actuatorId = msg.commands.data[idx].actuator_id;
      if (smartImmediateStatusPending[actuatorId]) {
	ServoSmart::publishImmediateStatus(actuatorId);
	smartImmediateStatusPending[actuatorId] = false;
      }
    }
  }
}

/** @brief Subscribe to actuator array command messages. */
DeviceStatus ServoRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<processActuatorArrayCommand>();
  return DeviceStatus::SERVO_ROLE;
}

/** @brief Start configured servo backends. */
DeviceStatus ServoRole::start(UAVCAN::Node& node)
{
  DeviceStatus status(DeviceStatus::SERVO_ROLE);
  
  m_role_servo_pwm = param_cget<"ROLE.servo.pwm">();
  m_role_servo_smart = param_cget<"ROLE.servo.smart">();

  if (m_role_servo_pwm)
    status = ServoPWM::start();

  if (m_role_servo_smart && status)
    status = ServoSmart::start(node);
 

  return status;
}

bool ServoRole::m_role_servo_pwm = false;
bool ServoRole::m_role_servo_smart = false;

#endif // USE_SERVO_ROLE
