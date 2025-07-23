#include "servoRole.hpp"
#include <algorithm>
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "servoPwm.hpp"
#include "servoSmart.hpp"
#include "stdutil++.hpp"



void ServoRole::processActuatorArrayCommand(CanardRxTransfer *,
					    const  uavcan_equipment_actuator_ArrayCommand &msg)
{
  for (size_t idx = 0; idx < msg.commands.len; ++idx) {
    if (ServoRole::m_role_servo_pwm) {
      if (msg.commands.data[idx].command_type ==
	  UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM) {
	ServoPWM::setPwm(msg.commands.data[idx].actuator_id,
			 msg.commands.data[idx].command_value);
      } else if (msg.commands.data[idx].command_type ==
		 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
	ServoPWM::setUnitless(msg.commands.data[idx].actuator_id,
			      msg.commands.data[idx].command_value);
      }
    }
      
    if (ServoRole::m_role_servo_smart) {
      if (msg.commands.data[idx].command_type ==
	  UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
	ServoSmart::setUnitless(msg.commands.data[idx].actuator_id,
				msg.commands.data[idx].command_value);
      } else if (msg.commands.data[idx].command_type ==
		 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_FORCE) {
	ServoSmart::setTorque(msg.commands.data[idx].actuator_id,
			      msg.commands.data[idx].command_value);
      } else if (msg.commands.data[idx].command_type ==
		 UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_SPEED) {
	ServoSmart::setSpeed(msg.commands.data[idx].actuator_id,
			     msg.commands.data[idx].command_value);
      } 
    }
  }
}


DeviceStatus ServoRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<processActuatorArrayCommand>();
  return DeviceStatus::SERVO_ROLE;
}

DeviceStatus ServoRole::start(UAVCAN::Node&)
{
  DeviceStatus status(DeviceStatus::SERVO_ROLE);
  
  m_role_servo_pwm = PARAM_CGET("role.servo.pwm");
  m_role_servo_smart = PARAM_CGET("role.servo.smart");

  if (m_role_servo_pwm)
    status = ServoPWM::start();

  if (m_role_servo_smart && status)
    status = ServoSmart::start();
 

  return status;
}

bool ServoRole::m_role_servo_pwm = false;
bool ServoRole::m_role_servo_smart = false;
