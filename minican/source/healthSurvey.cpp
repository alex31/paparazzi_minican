#include "healthSurvey.hpp"
#include <ch.h>
#include <algorithm>
#include <limits>
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"
#include "UAVCAN/persistantParam.hpp"

namespace  {
  THD_WORKING_AREA(waAdcSampling, 512) __attribute__((section(FAST_SECTION "_clear")));
[[noreturn]]
  void adcSampling (void *arg);
}


namespace HealthSurvey {
  void start(UAVCAN::Node& node) {
    chThdCreateStatic(waAdcSampling, sizeof(waAdcSampling), NORMALPRIO, &adcSampling, &node);
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
    chRegSetThreadName("adcSampling");
    
    while (true) {
      const float coreTempC = Adc::getCoreTemp();
      const float batteryVoltage = Adc::getPsBat();
      const float vcc = Adc::getVcc();

      const float currentNaN = std::numeric_limits<float>::quiet_NaN();
      const uint8_t cells = static_cast<uint8_t>(
        std::clamp<int>(param_cget<"role.voltmeter.cells">(), 2, 6));

      static constexpr float kCellOverVoltage = 4.2f;
      static constexpr float kCellUnderVoltage = 3.4f;
      const float batOver = kCellOverVoltage * cells;
      const float batUnder = kCellUnderVoltage * cells;

      tempMsg.device_id = 0;
      tempMsg.temperature = coreTempC + 273.15f;
      tempMsg.error_flags = 0;

      batteryMsg.circuit_id = 0;
      batteryMsg.voltage = batteryVoltage;
      batteryMsg.current = currentNaN;
      batteryMsg.error_flags = 0;
      if (batteryVoltage > batOver) {
        batteryMsg.error_flags |= UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_OVERVOLTAGE;
      }
      if (batteryVoltage < batUnder) {
        batteryMsg.error_flags |= UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_UNDERVOLTAGE;
      }

      vccMsg.circuit_id = 1;
      vccMsg.voltage = vcc;
      vccMsg.current = currentNaN;
      vccMsg.error_flags = 0;
      if (vcc > ps3vMax) {
        vccMsg.error_flags |= UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_OVERVOLTAGE;
      }
      if (vcc < ps3vMin) {
        vccMsg.error_flags |= UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ERROR_FLAG_UNDERVOLTAGE;
      }

      node->sendBroadcast(tempMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      node->sendBroadcast(batteryMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      node->sendBroadcast(vccMsg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      chThdSleepMilliseconds(500);
    }
  }
  
}
