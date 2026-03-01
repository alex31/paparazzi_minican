/**
 * @file servoSmart.cpp
 * @brief Smart servo role implementation for STS3032 devices.
 */
#include "roleConf.h"

#if USE_SERVO_ROLE

#include "servoSmart.hpp"
#include "resourceManager.hpp"
#include "smart_servos/STS3032.h"
#include "hardwareConf.hpp"
#include "UAVCanHelper.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <limits>


#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

namespace  {
  SIOConfig servoSioCfg = {
    .baud = 250'000,
    .presc = USART_PRESC1,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS,
    .cr3 = USART_CR3_HDSEL
  };

  constexpr SIO::DmaUserConfig servo_rx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = EXTERNAL_USART_RX_DMAMUX,
  };
  constexpr SIO::DmaUserConfig servo_tx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = EXTERNAL_USART_TX_DMAMUX,
  };

  alignas(SIO::Datagram) static uint8_t servo_sio_storage[sizeof(SIO::Datagram)];
  SIO::Datagram *servoSio = nullptr;
  alignas(STS3032) static uint8_t servo_bus_storage[sizeof(STS3032)];
  STS3032 *servoBus = nullptr;
  uint32_t	     startIndex = std::numeric_limits<uint32_t>::max();
  uint32_t	     numServos = 0;
  uint32_t	     reportPeriod = 0;
  constexpr sysinterval_t kImmediateStatusMinInterval = TIME_MS2I(8);
  std::array<systime_t, BROADCAST_ID + 1U> immediateLastPublishTs = {};
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
  immediateLastPublishTs.fill(0U);
  nodep = &node;

  // use serial2 TX + RX
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA09, HR::F2)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_1));
  }
  // MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b00100);
#endif

  
  if (servoSio == nullptr) {
    const SIO::DatagramConfig cfg = {
      ExternalSIOD,
      servo_rx_dma_cfg,
      servo_tx_dma_cfg,
      servoSioCfg
    };
    servoSio = new (servo_sio_storage) SIO::Datagram(cfg);
  }
  if (servoBus == nullptr) {
    servoBus = new (servo_bus_storage) STS3032(servoSio, &servoSioCfg);
  }

  servoBus->init();
  if (auto status = servoBus->detectBaudrate({1'000'000U, 500'000U, 250'000U}); status == SmartServo::OK) {
    DebugTrace("detectBaudrate OK -> Kbaud = %lu",
	       servoBus->getSerialBaudrate() / 1000U);
  } else {
    DebugTrace("detectBaudrate failed with status 0x%x: aborting", status);
    if (status == SmartServo::STATUS_TIMEOUT) {
      return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::NOT_RESPONDING, static_cast<uint16_t>(status));
    }
    if (status == SmartServo::HETEROGENEOUS_BAUDRATES) {
      return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::HETEROGENEOUS_BAUDS, static_cast<uint16_t>(status));
    }
    return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::INVALID_PARAM, static_cast<uint16_t>(status));
  }

  if ((startIndex + numServos) > BROADCAST_ID) {
    DebugTrace("Error: invalid smart-servo mapping start=%lu count=%lu", startIndex, numServos);
    return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::INVALID_PARAM, startIndex);
  }

  bool responding[BROADCAST_ID + 1U] = {false};
  uint32_t respondingCount = 0U;
  for (uint16_t id = 0U; id < BROADCAST_ID; id++) {
    if (servoBus->ping(static_cast<uint8_t>(id)) == SmartServo::OK) {
      responding[id] = true;
      respondingCount++;
      DebugTrace("smart-servo scan: id=%u responding", id);

      char text[90];
      std::snprintf(text, sizeof(text), "servo.smart: id %u responding", static_cast<unsigned>(id));
      (void)UAVCAN::Helper::log(node, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO,
				"servoSmart.cpp::start()", text);
    }
  }
  DebugTrace("smart-servo scan: %lu id(s) responding", respondingCount);
  if (respondingCount == 0U) {
    return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::NOT_RESPONDING, 0U);
  }

  for (uint32_t id = startIndex; id < (startIndex + numServos); id++) {
    if (!responding[id]) {
      DebugTrace("Error: configured servo id=%lu not responding", id);
      return DeviceStatus(DeviceStatus::SERVO_SMART, DeviceStatus::NOT_RESPONDING, id);
    }
    setUnitless(id, 0);
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
    servoBus->move(index, pos);
  } 
}

/** @brief Command a servo torque limit using a normalized input. */
void ServoSmart::setTorque(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t torque = remap<0.0f, 1.0f, 0.0f, 1e4f>(value);
    servoBus->setTorque(index, torque);
    servoBus->torqueEnable(index, torque != 0U);
  } 
}

/** @brief Command a servo speed limit using radians per second. */
void ServoSmart::setSpeed(uint8_t index, float value)
{
  // value in Rad/second
  // speedLimit in : Nb of steps/second. 50 steps / second = 0.732 RPM
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    uint16_t speed = std::min(65535.0f, value * 652.7f);   
    servoBus->speedLimit(index, speed);
  } 
}

/** @brief Publish one status sample immediately, with per-servo rate limiting. */
void ServoSmart::publishImmediateStatus(uint8_t index)
{
  if (index > BROADCAST_ID || nodep == nullptr || servoBus == nullptr) {
    return;
  }
  if ((startIndex > index) || (index >= (startIndex + numServos))) {
    return;
  }

  const systime_t now = chVTGetSystemTimeX();
  systime_t& last = immediateLastPublishTs[index];
  if ((last != 0U) && (chTimeDiffX(last, now) < kImmediateStatusMinInterval)) {
    // DebugTrace("smart-servo immediate status rate-limited id=%u", index);
    return;
  }
  last = now;

  STS3032::StateVector sv = servoBus->readStates(index);
  if (sv.status == SmartServo::STATUS_TIMEOUT) {
    // DebugTrace("smart-servo immediate status read timeout id=%u", index);
    return;
  }

  uavcan_equipment_actuator_Status msg;
  copy(sv, msg, index);
  const auto canStatus = nodep->sendBroadcast(msg);
  if (canStatus == UAVCAN::Node::CAN_OK) {
    // DebugTrace("smart-servo immediate status published id=%u", index);
  } else {
    // DebugTrace("smart-servo immediate status publish failed id=%u status=%u",
    //	       index, static_cast<unsigned>(canStatus));
  }
}

namespace {
  /** @brief Periodically poll servo states and publish UAVCAN status messages. */
  void periodic(void *)
  {
     while (true) {
       systime_t ts = chVTGetSystemTimeX();
       for(uint8_t id = startIndex; id < startIndex + numServos; id++) {
	 STS3032::StateVector sv = servoBus->readStates(id);
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
