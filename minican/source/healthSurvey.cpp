#include "healthSurvey.hpp"
#include <ch.h>
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"

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
    mppt_Stream msg = {};
    UAVCAN::Node *node = static_cast<UAVCAN::Node *>(arg);
    chRegSetThreadName("adcSampling");
    
    while (true) {
      msg.temperature = Adc::getCoreTemp();
      msg.input_voltage = Adc::getPsBat();
      msg.output_voltage = Adc::getVcc();
  
      msg.fault_flags = 0;
      if (msg.temperature > maxCoreTemp)
	msg.fault_flags |= MPPT_STREAM_OT_FAULT;
      if ((msg.input_voltage > psBatMax) or (msg.output_voltage >  ps3vMax))
	msg.fault_flags |= MPPT_STREAM_OV_FAULT;
      if ((msg.input_voltage < psBatMin) or (msg.output_voltage <  ps3vMin))
	msg.fault_flags |= MPPT_STREAM_UV_FAULT;

      // if (msg.fault_flags) {
      // 	DebugTrace("T = %d", msg.temperature);
      // 	DebugTrace("5V = %.2f", msg.input_voltage);
      // 	DebugTrace("3V = %.2f", msg.output_voltage);
      // }
      
      node->sendBroadcast(msg, CANARD_TRANSFER_PRIORITY_MEDIUM);
      chThdSleepMilliseconds(500);
    }
  }
  
}
