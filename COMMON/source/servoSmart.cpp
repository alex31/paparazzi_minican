#include "servoSmart.hpp"
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "smart_servos/STS3032.h"
#include "stdutil++.hpp"
#include "hardwareConf.hpp"

/*
  • Review – servoSmart.cpp status publishing

  - COMMON/source/servoSmart.cpp:113 – status loop always iterates id = 1..numServos and ignores startIndex, so if your servo map doesn’t start at 1 you’ll publish
    IDs that don’t match the command mapping (and you’ll query non-existent IDs on the bus).
  - COMMON/source/servoSmart.cpp:128-131 – position, speed, power_rating_pct are published without unit conversion/clamping. STS3032::StateVector uses normalized
    [-1..1] for position and steps/s for speed; 1011.Status expects radians and rad/s. power_rating_pct is a uint7 and should be clamped 0..100 (or 127) with UNKNOWN
    when unavailable.
  - COMMON/source/servoSmart.cpp:129 – Unavailable fields should be NaN/UNKNOWN per the DSDL comment; setting force = -1 misleads consumers into thinking the actuator
    is applying -1 N·m/-1 N.
  - COMMON/source/servoSmart.cpp:113-118 – No error handling when readStates() fails/timeouts; you’ll broadcast stale defaults (e.g., zeroed struct with
    status=STATUS_TIMEOUT) as valid statuses and may block the loop if a servo is offline.

  Suggested next steps: honor startIndex when iterating/publishing; convert normalized position/load/speed into physical units (or publish NaN/UNKNOWN when not
  available); clamp power_rating_pct; use NaN for unknown force; add handling for read failures (e.g., skip publish or mark UNKNOWN).
 

 */

#ifdef     BOARD_ENAC_MICROCANv3
#include "dynamicPinConfig.hpp"
#endif

namespace  {
  STS3032 servoBus(&ExternalUARTD);
  uint32_t	     startIndex = std::numeric_limits<uint32_t>::max();
  uint32_t	     numServos = 0;
  uint32_t	     reportPeriod = 0;
  void periodic(void *);
  UAVCAN::Node*	     nodep = nullptr;
  void copy(const STS3032::StateVector& sv, uavcan_equipment_actuator_Status &msg,
	    uint8_t id);
}


DeviceStatus ServoSmart::start(UAVCAN::Node& node)
{
  using HR = HWResource;
  startIndex = PARAM_CGET("role.servo.smart.map_index1");
  numServos =  PARAM_CGET("role.servo.smart.num_servos");
  reportPeriod = CH_CFG_ST_FREQUENCY / PARAM_CGET("role.servo.smart.status_frequency");
  nodep = &node;
    
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

  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1536), "smart servos periodic", NORMALPRIO, 
		      periodic, nullptr);
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

namespace {
  
  void periodic(void *)
  {
     while (true) {
       systime_t ts = chVTGetSystemTimeX();
       for(uint8_t id = startIndex; id < startIndex + numServos; id++) {
	 STS3032::StateVector sv = servoBus.readStates(id);
	 if (sv.status != SmartServo::STATUS_TIMEOUT) {
	   uavcan_equipment_actuator_Status msg;
	   copy(sv, msg, id);
	   nodep->sendBroadcast(msg);
	 }
       }
       chThdSleepUntilWindowed(ts, ts + reportPeriod);
      }
  }


  void copy(const STS3032::StateVector& sv, uavcan_equipment_actuator_Status &msg,
	    uint8_t id)
  {
    msg.actuator_id = id;
    msg.position = sv.position;
    msg.force = std::numeric_limits<float>::quiet_NaN(); // not available
    msg.speed = sv.speed;
    msg.power_rating_pct = std::clamp(static_cast<int>(sv.load * 100), 0, 100);
  }

  
}
