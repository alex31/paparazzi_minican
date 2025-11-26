#include "servoSmart.hpp"
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "smart_servos/STS3032.h"
#include "stdutil++.hpp"
#include "hardwareConf.hpp"



#ifdef     BOARD_ENAC_MICROCANv3
#include "dynamicPinConfig.hpp"
#endif

namespace  {
  STS3032 servoBus(&ExternalUARTD);
  uint32_t	     startIndex = std::numeric_limits<uint32_t>::max();
  uint32_t	     numServos = 0;
}


DeviceStatus ServoSmart::start()
{
  using HR = HWResource;
  startIndex = PARAM_CGET("role.servo.smart.map_index1");
  numServos =  PARAM_CGET("role.servo.smart.num_servos");

#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA09, HR::PA10,
				   HR::F2, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  // MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b01100);
#endif

  
  //  PWMChannelConfig
  servoBus.init();
  if (auto status = servoBus.detectBaudrate({1'000'000U, 500'000U, 250'000U}); status == SmartServo::OK) {
    DebugTrace("detectBaudrate OK -> Kbaud = %lu",
	       servoBus.getSerialBaudrate() / 1000U);
  } else {
    DebugTrace("detectBaudrate failed with status 0x%x: aborting", status);
    return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::HETEROGENEOUS_BAUDS);
  }
  
    
  for(uint8_t id = 1U; id <= numServos; id++) {
    if (servoBus.ping(id) == SmartServo::OK) {
      DebugTrace("ping ok return id = %u", id);
      setUnitless(id, 0);
    } else {
      DebugTrace("Error : servo %u not responding", id);
      return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::NOT_RESPONDING, id);
    }
  }

  return DeviceStatus(DeviceStatus::SERVO_SMART);
}

/*
  index : first_index ..  (first_index + num_servos - 1);
  value IN  : -1.0 : 1.0 
  value OUT : 0 .. 4095 
 */
void ServoSmart::setUnitless(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t pos = remap<-1.0f, 1.0f, 0.0f, 4095.0f>(value);
    servoBus.move(index, pos);
  } 
}

void ServoSmart::setTorque(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t torque = remap<0.0f, 1.0f, 0.0f, 1e4f>(value);
    servoBus.setTorque(index, torque);
    servoBus.torqueEnable(index, torque != 0U);
  } 
}

void ServoSmart::setSpeed(uint8_t index, float value)
{
  // value in Rad/second
  // speedLimit in : Nb of steps/second. 50 steps / second = 0.732 RPM
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t speed = std::min(65535.0f, value * 652.7f);   
    servoBus.speedLimit(index, speed);
  } 
}

