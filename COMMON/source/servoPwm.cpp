#include "servoPwm.hpp"
#include <algorithm>
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil++.hpp"



#ifdef     BOARD_ENAC_MICROCANv1
#include "dynamicPinConfig.hpp"
#endif

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)

#ifdef BOARD_ENAC_MINICANv4
static_assert(SRV1_TIM_CH == 1, "assume SRV1 is on CH1");
static_assert((SRV2_TIM == SRV1_TIM) && (SRV2_TIM_CH == 2), "assume SRV2 is on CH2 and all Ch on same timer");
static_assert((SRV3_TIM == SRV1_TIM) && (SRV3_TIM_CH == 3), "assume SRV3 is on CH3 and all Ch on same timer");
static_assert((SRV4_TIM == SRV1_TIM) && (SRV4_TIM_CH == 4), "assume SRV4 is on CH4 and all Ch on same timer");
static constexpr PWMDriver &SERVO_PWMD =  CONCAT(PWMD, SRV1_TIM);
#elifdef     BOARD_ENAC_MICROCANv1
static_assert(F1_b_TIM_CH == 4, "assume SRV4 is on F1");
static_assert((F2_b_TIM == F1_b_TIM) && (F2_b_TIM_CH == 3), "assume SRV2 is on F2 and all Ch on same timer");
static_assert((F3_b_TIM == F1_b_TIM) && (F3_b_TIM_CH == 2), "assume SRV3 is on F3 and all Ch on same timer");
static_assert((F4_b_TIM == F1_b_TIM) && (F4_b_TIM_CH == 1), "assume SRV4 is on F4 and all Ch on same timer");
static constexpr PWMDriver &SERVO_PWMD =  CONCAT(PWMD, F1_b_TIM);
#endif



namespace  {
  constexpr uint32_t servoTickFreq = 2'000'000U;
  constinit uint32_t servoPwmFreq  =  50U;
  constexpr uint32_t servoTicksPerPeriod = 40000; // servoTickFreq / servoPwmFreq
  float              halfWidthFactor = 1.0f;
  uint32_t	     startIndex = std::numeric_limits<uint32_t>::max();
  uint32_t	     numServos = 0;

  PWMConfig pwmServoCfg = {     
    .frequency = servoTickFreq,        
    .period    = servoTicksPerPeriod, 
    .callback  = NULL,           
    .channels  = {}, // all channels disabled : enable depending on role.pwm.num_servos
    .cr2  = 0, 
    .bdtr = 0,
    .dier = 0  
  };

}


DeviceStatus ServoPWM::start()
{
  using HR = HWResource;
  servoPwmFreq = PARAM_CGET("role.servo.pwm.frequency");
  chDbgAssert(servoPwmFreq >= 50, "invalid servoPwmFreq parameter");
  halfWidthFactor = PARAM_CGET("role.servo.pwm.pulse_half_width") ? 1.0f : 2.0f;
  startIndex = PARAM_CGET("role.servo.pwm.map_index1");
  numServos =  PARAM_CGET("role.servo.pwm.num_servos");


#ifdef     BOARD_ENAC_MICROCANv1
  DynPin::setScenario(DynPin::Scenario::PWM);
#endif
  pwmServoCfg.period = servoTickFreq / servoPwmFreq;

  // use timer
  if (not boardResource.try_acquire(HR::TIM_1)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }

  // only the pins that are actually in use depending on role.pwm.num_servos
  for (uint8_t channel=0; channel < numServos; channel++) {
    pwmServoCfg.channels[channel] = {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL};
    if (not boardResource.try_acquire(static_cast<HR>(std::to_underlying(HR::PA08) + channel))) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
    }
  }
  
  //  PWMChannelConfig
  pwmStart(&SERVO_PWMD, &pwmServoCfg);
  for (uint8_t srv = startIndex; srv < (startIndex + numServos); srv++) {
    setPwm(srv, 500U);
  }
  return DeviceStatus(DeviceStatus::SERVO_PWM);
}

/*
  index : first_index ..  (first_index + num_servos - 1);
  value : 0 .. 1000
 
  2Mhz : 2.000 pour 1ms 4000 pour 2ms sauf si halfpatata
  index=0    -> channel = 2000 (ou 1000 si half_width)
  index=1000 -> channel = 4000 (ou 2000 si half_width)
 */
void ServoPWM::setPwm(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    float pulseWidth = remap<0.0f, 1000.0f, 1000.0f, 2000.0f>(value);
    pwmEnableChannel(&SERVO_PWMD,
		     index - startIndex,
		     pulseWidth * halfWidthFactor
		     );
  } 
}
/*
  index : first_index ..  (first_index + num_servos - 1);
  value : -1.0 : 1.0 
 
  2Mhz : 2.000 pour 1ms 4000 pour 2ms sauf si halfpatata
  index=0    -> channel = 2000 (ou 1000 si half_width)
  index=1000 -> channel = 4000 (ou 2000 si half_width)
 */
void ServoPWM::setUnitless(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + numServos))) {
    float pulseWidth = remap<-1.0f, 1.0f, 1000.0f, 2000.0f>(value);
    pwmEnableChannel(&SERVO_PWMD,
		     index - startIndex,
		     pulseWidth * halfWidthFactor
		     );
  } 
}
