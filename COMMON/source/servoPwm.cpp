#include "servoPwm.hpp"
#include <algorithm>
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil++.hpp"



#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)

#if PLATFORM_MINICAN
static_assert(SRV1_TIM_CH == 1, "assume SRV1 is on CH1");
static_assert((SRV2_TIM == SRV1_TIM) && (SRV2_TIM_CH == 2), "assume SRV2 is on CH2 and all Ch on same timer");
static_assert((SRV3_TIM == SRV1_TIM) && (SRV3_TIM_CH == 3), "assume SRV3 is on CH3 and all Ch on same timer");
static_assert((SRV4_TIM == SRV1_TIM) && (SRV4_TIM_CH == 4), "assume SRV4 is on CH4 and all Ch on same timer");
static constexpr PWMDriver &SERVO_PWMD =  CONCAT(PWMD, SRV1_TIM);
#elif PLATFORM_MICROCAN
static_assert(F1_b_TIM_CH == 1, "assume SRV1 is on F1");
static_assert((F2_a_TIM == F1_b_TIM) && (F2_a_TIM_CH == 2), "assume SRV2 is on F2 and all Ch on same timer");
static_assert((F3_TIM == F1_b_TIM) && (F3_TIM_CH == 3), "assume SRV3 is on F3 and all Ch on same timer");
static_assert((F4_TIM == F1_b_TIM) && (F4_TIM_CH == 4), "assume SRV4 is on F4 and all Ch on same timer");
static constexpr PWMDriver &SERVO_PWMD =  CONCAT(PWMD, F1_b_TIM);
#endif



namespace  {
  struct ChannelMap {
    std::array<std::uint8_t, 8> idx{};
    std::uint8_t count{};
    std::uint8_t mask{};
    constexpr ChannelMap() = default;
    constexpr ChannelMap(std::uint8_t _mask) { *this = _mask; }
    
    constexpr ChannelMap& operator=(std::uint8_t _mask) {
      mask = _mask;
      count = 0;
      for (std::uint8_t bit = 0; bit < 8; ++bit) {
	if (mask & (1u << bit)) {
	  idx[count++] = bit;
	}
      }
      return *this;
    }
    
    constexpr std::uint8_t operator[](std::size_t i) const {
      chDbgAssert(i < count, "buffer overflow");
      return idx[i];
    }
  };
  
  
  constexpr uint32_t servoTickFreq = 2'000'000U;
  constinit uint32_t servoPwmFreq  =  50U;
  constexpr uint32_t servoTicksPerPeriod = 40000; // servoTickFreq / servoPwmFreq
  float              halfWidthFactor = 1.0f;
  uint32_t	     startIndex = std::numeric_limits<uint32_t>::max();
  ChannelMap	     channelMap;
  
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
  channelMap = PARAM_CGET("role.servo.pwm.channel_mask");
  
  if (channelMap.count > 4) {
    return DeviceStatus(DeviceStatus::SERVO_PWM, DeviceStatus::INVALID_PWM_MASK);
  }


#if PLATFORM_MICROCAN
  DynPin::setScenario(DynPin::Scenario::PWM, channelMap.mask);
#endif
  pwmServoCfg.period = servoTickFreq / servoPwmFreq;

  // use timer
  if (not boardResource.tryAcquire(HR::TIM_1)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }

  // only the pins that are actually in use depending on role.pwm.num_servos
  for (uint8_t channelIdx=0; channelIdx < channelMap.count; channelIdx++) {
    uint8_t channel = channelMap[channelIdx];
    pwmServoCfg.channels[channel] = {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL};
    if (not boardResource.tryAcquire(static_cast<HR>(std::to_underlying(HR::PA08) + channel))) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
    }
#if PLATFORM_MICROCAN
    if (not boardResource.tryAcquire(static_cast<HR>(std::to_underlying(HR::F1) + channel))) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
    }
#endif      
  }
  
  //  PWMChannelConfig
  pwmStart(&SERVO_PWMD, &pwmServoCfg);
  for (uint8_t srv = startIndex; srv < (startIndex + channelMap.count); srv++) {
    setPwm(srv, 1500.0f);
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
void ServoPWM::setPwm(uint8_t index, float pulseWidth /*value*/)
{
  if ((startIndex <= index) and (index < (startIndex + channelMap.count))) {
    // float pulseWidth = remap<0.0f, 1000.0f, 1000.0f, 2000.0f>(value);
    pwmEnableChannel(&SERVO_PWMD,
		     channelMap[index - startIndex],
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
  if ((startIndex <= index) and (index < (startIndex + channelMap.count))) {
    float pulseWidth = remap<-1.0f, 1.0f, 1000.0f, 2000.0f>(value);
    pwmEnableChannel(&SERVO_PWMD,
		     channelMap[index - startIndex],
		     pulseWidth * halfWidthFactor
		     );
  } 
}
