#include "healthSurvey.hpp"
#include <ch.h>
#include <algorithm>
#include <limits>
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"
#include "UAVCAN/persistantParam.hpp"

namespace  {
  [[noreturn]]
  void adcSampling (void *arg);
}


namespace HealthSurvey {
  void start(UAVCAN::Node& node) {
    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "health survey", NORMALPRIO, 
			&adcSampling, &node);
  }
}


namespace  {
  [[noreturn]]
  void adcSampling (void *arg)
  {
    uavcan_equipment_device_Temperature tempMsg = {};
    uavcan_equipment_power_CircuitStatus batteryMsg = {};
    uavcan_equipment_power_CircuitStatus vccMsg = {};
    UAVCAN::Node *node = static_cast<UAVCAN::Node *>(arg);
    batteryMsg.current = vccMsg.current = std::numeric_limits<float>::quiet_NaN();
    batteryMsg.circuit_id = 0;
    vccMsg.circuit_id = 1;
    tempMsg.device_id = 0;
    auto cells = param_cget<"role.voltmeter.cells">();
    static constexpr float kCellOverVoltage = 4.2f;
    static constexpr float kCellUnderVoltage = 3.4f;
    const float batOver = kCellOverVoltage * cells;
    const float batUnder = kCellUnderVoltage * cells;
    
    while (true) {
      const float coreTempC = Adc::getCoreTemp();
      const float batteryVoltage = Adc::getPsBat();
      const float vcc = Adc::getVcc();
      
      tempMsg.temperature = coreTempC + 273.15f;
      if (coreTempC > coreTempMax) {
	tempMsg.error_flags = UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ERROR_FLAG_OVERHEATING;
      } else if (coreTempC < -coreTempMin) {
	tempMsg.error_flags = UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ERROR_FLAG_OVERCOOLING;
      } else {
	tempMsg.error_flags = 0;
      }
      
      batteryMsg.voltage = batteryVoltage;
      if (batteryVoltage > batOver) {
        batteryMsg.error_flags = UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_OVERVOLTAGE;
      } else if (batteryVoltage < batUnder) {
        batteryMsg.error_flags = UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_UNDERVOLTAGE;
      } else {
	batteryMsg.error_flags = 0;
      }
      
      vccMsg.voltage = vcc;
      if (vcc > ps3vMax) {
        vccMsg.error_flags = UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_OVERVOLTAGE;
      } else if (vcc < ps3vMin) {
        vccMsg.error_flags = UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_UNDERVOLTAGE;
      } else {
	vccMsg.error_flags = 0;
      }
      
      node->sendBroadcast(tempMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      node->sendBroadcast(batteryMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      node->sendBroadcast(vccMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      chThdSleepMilliseconds(1000);
    }
  }
  
}
