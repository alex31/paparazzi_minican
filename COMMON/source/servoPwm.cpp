/**
 * @file servoPwm.cpp
 * @brief PWM servo output implementation.
 */
#include "roleConf.h"

#if USE_SERVO_ROLE

#include "servoPwm.hpp"
#include "resourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"

namespace  {
  struct LogicalChannelDef {
    PWMDriver* driver;
    pwmchannel_t hwChannel;
    HWResource timerRes;
    HWResource pinRes;
    ioline_t line;
    uint32_t af;
  };

  const LogicalChannelDef LOGICAL_CHANNELS[7] = {
    {&PWMD1, 0, HWResource::TIM_1, HWResource::PA08, LINE_SRV1, SRV1_TIM_AF},
    {&PWMD1, 1, HWResource::TIM_1, HWResource::PA09, LINE_SRV2, SRV2_TIM_AF},
    {&PWMD1, 2, HWResource::TIM_1, HWResource::PA10, LINE_SRV3, SRV3_TIM_AF},
    {&PWMD1, 3, HWResource::TIM_1, HWResource::PA11, LINE_SRV4, SRV4_TIM_AF},
    {&PWMD3, 0, HWResource::TIM_3, HWResource::PB04, LINE_UART_RX, UART_RX_TIM_AF},
    {&PWMD3, 1, HWResource::TIM_3, HWResource::PA04, LINE_SPI_PERIPH_CS, SPI_PERIPH_CS_TIM_AF},
    {&PWMD3, 3, HWResource::TIM_3, HWResource::PB07, LINE_I2C_SDA, I2C_SDA_TIM_AF},
  };

  /** @brief Map an enable mask to a packed list of PWM channels. */
  struct ChannelMap {
    std::array<std::uint8_t, 8> idx{};
    std::uint8_t count{};
    std::uint8_t mask{};
    constexpr ChannelMap() = default;
    constexpr ChannelMap(std::uint8_t _mask) { *this = _mask; }
    
    constexpr ChannelMap& operator=(std::uint8_t _mask) {
      mask = _mask;
      count = 0;
      for (std::uint8_t bit = 0; bit < 7; ++bit) { // max 7 channels
        if (mask & (1u << bit)) {
          idx[count++] = bit;
        }
      }
      return *this;
    }
    
    constexpr std::uint8_t operator[](std::size_t i) const {
      return i < count ? idx[i] : 0;
    }
  };

  // 170 MHz / 64 = 2.65625 MHz is the highest exact PWM base that still keeps
  // a 50 Hz period within a 16-bit timer (53125 ticks).
  constexpr uint32_t servoTickFreq = 2'656'250U;
  constexpr uint32_t servoTicksPerPeriod = servoTickFreq / 50U;

  struct TimerSettings {
    uint32_t freq;
    float pulseScaleFactor;
  };

  constexpr float regularPwmScaleFactor = 2.65625f;     // 2.65625 ticks/us.
  constexpr float shot125ScaleFactor = 0.33203125f;    // regular scale divided by 8.

  TimerSettings tim1Settings = {50U, regularPwmScaleFactor};
  TimerSettings tim3Settings = {50U, regularPwmScaleFactor};

  uint32_t startIndex = std::numeric_limits<uint32_t>::max();
  ChannelMap channelMap;

  PWMConfig makeDisabledPwmConfig() {
    return {     
      .frequency = servoTickFreq,        
      .period    = servoTicksPerPeriod, 
      .callback  = NULL,           
      .channels  = {}, // all channels disabled
      .cr2  = 0, 
      .bdtr = 0,
      .dier = 0  
    };
  }

  PWMConfig pwmServoCfgTim1;
  PWMConfig pwmServoCfgTim3;

  pwmcnt_t pulseWidthUsToTicks(float pulseWidthUs, float scaleFactor)
  {
    return static_cast<pwmcnt_t>((pulseWidthUs * scaleFactor) + 0.5f);
  }

  void configureTim3PwmLine(const LogicalChannelDef& def)
  {
    if (def.timerRes != HWResource::TIM_3) {
      return;
    }
    palSetLineMode(def.line, PAL_MODE_ALTERNATE(def.af) | PAL_STM32_OSPEED_HIGHEST);
  }
}

/** @brief Initialize PWM timers and configure active servo channels. */
DeviceStatus ServoPWM::start()
{
  using HR = HWResource;

  tim1Settings.freq = param_cget<"role.servo.pwm.ch1-4.frequency">();
  bool shot125Tim1 = param_cget<"role.servo.pwm.ch1-4.shot125">();
  tim3Settings.freq = param_cget<"role.servo.pwm.ch5-7.frequency">();
  bool shot125Tim3 = param_cget<"role.servo.pwm.ch5-7.shot125">();

  startIndex = param_cget<"role.servo.pwm.map_index1">();
  channelMap = param_cget<"role.servo.pwm.channel_mask">();
  
  if (channelMap.count == 0 || channelMap.count > 7) {
    return DeviceStatus(DeviceStatus::SERVO_PWM, DeviceStatus::INVALID_PWM_MASK);
  }

  const bool useTim1 = (channelMap.mask & 0x0F) != 0; // CH1..CH4 on TIM1 (bits 0-3)
  const bool useTim3 = (channelMap.mask & 0x70) != 0; // CH5..CH7 on TIM3 (bits 4-6)

  // nodeParameters already enforces the absolute 50..1000 Hz range.
  // Keep only the mode-dependent complementary bound here.
  if (useTim1 && ((shot125Tim1 && (tim1Settings.freq < 400U)) || (!shot125Tim1 && (tim1Settings.freq > 400U)))) {
    return DeviceStatus(DeviceStatus::SERVO_PWM, DeviceStatus::INVALID_PARAM, tim1Settings.freq);
  }
  if (useTim3 && ((shot125Tim3 && (tim3Settings.freq < 400U)) || (!shot125Tim3 && (tim3Settings.freq > 400U)))) {
    return DeviceStatus(DeviceStatus::SERVO_PWM, DeviceStatus::INVALID_PARAM, tim3Settings.freq);
  }

  tim1Settings.pulseScaleFactor = shot125Tim1 ? shot125ScaleFactor : regularPwmScaleFactor;
  tim3Settings.pulseScaleFactor = shot125Tim3 ? shot125ScaleFactor : regularPwmScaleFactor;

  pwmServoCfgTim1 = makeDisabledPwmConfig();
  pwmServoCfgTim3 = makeDisabledPwmConfig();

  if (useTim1) {
    if (not boardResource.tryAcquire(HR::TIM_1)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT, std::to_underlying(HR::TIM_1));
    }
    pwmServoCfgTim1.period = servoTickFreq / tim1Settings.freq;
  }

  if (useTim3) {
    if (not boardResource.tryAcquire(HR::TIM_3)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT, std::to_underlying(HR::TIM_3));
    }
    pwmServoCfgTim3.period = servoTickFreq / tim3Settings.freq;
  }

  for (uint8_t channelIdx=0; channelIdx < channelMap.count; channelIdx++) {
    uint8_t channel = channelMap[channelIdx];
    const auto& def = LOGICAL_CHANNELS[channel];

    if (not boardResource.tryAcquire(def.pinRes)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT, std::to_underlying(def.pinRes));
    }

    configureTim3PwmLine(def);

    if (def.timerRes == HR::TIM_1) {
      pwmServoCfgTim1.channels[def.hwChannel] = {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL};
    } else {
      pwmServoCfgTim3.channels[def.hwChannel] = {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL};
    }
  }
  
  if (useTim1) pwmStart(&PWMD1, &pwmServoCfgTim1);
  if (useTim3) pwmStart(&PWMD3, &pwmServoCfgTim3);

  for (uint8_t srv = startIndex; srv < (startIndex + channelMap.count); srv++) {
    setPwm(srv, 1500.0f);
  }
  return DeviceStatus(DeviceStatus::SERVO_PWM);
}

void ServoPWM::setPwm(uint8_t index, float pulseWidth /*value*/)
{
  if ((startIndex <= index) and (index < (startIndex + channelMap.count))) {
    uint8_t logicalChannel = channelMap[index - startIndex];
    const auto& def = LOGICAL_CHANNELS[logicalChannel];
    float factor = (def.timerRes == HWResource::TIM_1) ? tim1Settings.pulseScaleFactor : tim3Settings.pulseScaleFactor;
    pwmEnableChannel(def.driver, def.hwChannel, pulseWidthUsToTicks(pulseWidth, factor));
  } 
}

void ServoPWM::setUnitless(uint8_t index, float value)
{
  if ((startIndex <= index) and (index < (startIndex + channelMap.count))) {
    float pulseWidth = remap<-1.0f, 1.0f, 1000.0f, 2000.0f>(value);
    uint8_t logicalChannel = channelMap[index - startIndex];
    const auto& def = LOGICAL_CHANNELS[logicalChannel];
    float factor = (def.timerRes == HWResource::TIM_1) ? tim1Settings.pulseScaleFactor : tim3Settings.pulseScaleFactor;
    pwmEnableChannel(def.driver, def.hwChannel, pulseWidthUsToTicks(pulseWidth, factor));
  } 
}

#endif // USE_SERVO_ROLE
