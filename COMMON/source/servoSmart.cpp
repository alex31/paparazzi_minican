/**
 * @file servoSmart.cpp
 * @brief Smart servo role implementation for STS3032 devices.
 */
#include "roleConf.h"

#if USE_SERVO_ROLE

#include "servoSmart.hpp"
#include "resourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "smart_servos/STS3032.h"
#include "stdutil++.hpp"
#include "hardwareConf.hpp"


#if PLATFORM_MICROCAN
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


/** @brief Initialize the smart servo bus and optional status reporting. */
DeviceStatus ServoSmart::start(UAVCAN::Node& node)
{
  using HR = HWResource;
  startIndex = param_cget<"role.servo.smart.map_index1">();
  numServos =  param_cget<"role.servo.smart.num_servos">();
  if (const uint32_t reportFrequency =  param_cget<"role.servo.smart.status_frequency">();
      reportFrequency != 0) {
    reportPeriod = CH_CFG_ST_FREQUENCY / param_cget<"role.servo.smart.status_frequency">();
  }
  nodep = &node;

  // use serial2 TX + RX
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA09, HR::PA10,
				   HR::F2, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_1));
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

  if (reportPeriod != 0) {
    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1536), "smart servos periodic", NORMALPRIO, 
			periodic, nullptr);
  }
  return DeviceStatus(DeviceStatus::SERVO_SMART);
}

/**
 * @brief Command a servo position using a unitless [-1; 1] input.
 *
 * The value is mapped to the internal 0..4095 position range.
 */
void ServoSmart::setUnitless(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t pos = remap<-1.0f, 1.0f, 0.0f, 4095.0f>(value);
    servoBus.move(index, pos);
  } 
}

/** @brief Command a servo torque limit using a normalized input. */
void ServoSmart::setTorque(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t torque = remap<0.0f, 1.0f, 0.0f, 1e4f>(value);
    servoBus.setTorque(index, torque);
    servoBus.torqueEnable(index, torque != 0U);
  } 
}

/** @brief Command a servo speed limit using radians per second. */
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
  /** @brief Periodically poll servo states and publish UAVCAN status messages. */
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

  /** @brief Convert a servo state vector into a UAVCAN status message. */
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

#endif // USE_SERVO_ROLE
