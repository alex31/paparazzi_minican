#include "roleConf.h"

#if USE_LED2812_ROLE

#include "rgbLedRole.hpp"

#include <algorithm>
#include <array>
#include <variant>

#include "UAVCAN/persistantParam.hpp"
#include "led2812.hpp"
#include "resourceManager.hpp"
#include "hardwareConf.hpp"

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT3_NX(st1, st2, st3) st1 ## st2 ## st3
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)
#define CONCAT3(st1, st2, st3) CONCAT3_NX(st1, st2, st3)


namespace {
  // Derive bounds from the frozen parameter metadata to avoid duplicate constants.
  constexpr auto ledParam = Persistant::Parameter::cfind("role.led2812.led_number");
  constexpr size_t kMaxLeds = static_cast<size_t>(std::get<Persistant::Integer>(ledParam.second.max));
  constexpr systime_t kFramePeriod = TIME_MS2I(20);

  // Fixed hardware mapping: PB07 / TIM3_CH4
  static constexpr PWMDriver &ledPwm = LedStripPWMD;
  static constexpr uint32_t dmaMux = CONCAT3(STM32_DMAMUX1_TIM, LED2812_TIM, _UP);
  static constexpr LedTiming ledTiming = getClockByTimer(&ledPwm);
  using Led_t = Led2812<uint16_t, ledTiming.t0h, ledTiming.t1h>;

  Led2812Strip<kMaxLeds, Led_t> *ledStrip = nullptr;
  size_t ledCount = 1;
  std::array<RGB, kMaxLeds> desiredColors{};

  THD_WORKING_AREA(waLedStrip, 512);
  void ledThread(void *arg);

  RGB convertRgb565(const uavcan_equipment_indication_RGB565 &c)
  {
    // DSDL uses 5/6/5 bits even though the generated fields are uint8_t;
    // expand to full 8-bit channels.
    auto expand5 = [](uint8_t v) {
      return static_cast<uint8_t>((v << 3) | (v >> 2));
    };
    auto expand6 = [](uint8_t v) {
      return static_cast<uint8_t>((v << 2) | (v >> 4));
    };

    return {expand5(c.red), expand6(c.green), expand5(c.blue)};
  }

  void updateColor(uint8_t idx, const uavcan_equipment_indication_RGB565 &color)
  {
    if ((ledCount == 0) || (idx >= ledCount)) {
      return;
    }
    desiredColors[idx] = convertRgb565(color);
  }
}


DeviceStatus RgbLedRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  ledCount = std::clamp<size_t>(param_cget<"role.led2812.led_number">(), 1U, kMaxLeds);
  node.subscribeBroadcastMessages<Trampoline<&RgbLedRole::processLightsCommand>::fn>();
  return DeviceStatus::LED2812_ROLE;
}


DeviceStatus RgbLedRole::start(UAVCAN::Node& /*node*/)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::LED2812_ROLE);

  const size_t activeLedCount = std::clamp(ledCount, size_t{1}, kMaxLeds);

  const bool acquired =
#if PLATFORM_MICROCAN
    boardResource.tryAcquire(HR::TIM_3, HR::PB07, HR::F0);
#else
    boardResource.tryAcquire(HR::TIM_3, HR::PB07);
#endif

  if (not acquired) {
    const auto conflict =
      boardResource.isAllocated(HR::TIM_3) ? HR::TIM_3 :
#if PLATFORM_MICROCAN
      (boardResource.isAllocated(HR::F0) ? HR::F0 : HR::PB07);
#else
      HR::PB07;
#endif
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(conflict));
  }

#if PLATFORM_MINICAN
  palSetLineMode(LINE_I2C_SDA, PAL_MODE_ALTERNATE(I2C_SDA_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
#elif PLATFORM_MICROCAN
  palSetLineMode(LINE_F0_b, PAL_MODE_ALTERNATE(F0_b_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
#endif

  static Led2812Strip<kMaxLeds, Led_t> strip(&ledPwm, ledTiming,
					     STM32_DMA_STREAM_ID_ANY,
					     dmaMux,
					     static_cast<TimerChannel>(LED2812_TIM_CH - 1U),
					     activeLedCount);
  ledStrip = &strip;

  chThdCreateStatic(waLedStrip, sizeof(waLedStrip), NORMALPRIO,
		    &ledThread, nullptr);
  return status;
}


void RgbLedRole::processLightsCommand(CanardRxTransfer *,
				      const  uavcan_equipment_indication_LightsCommand &msg)
{
  const uint8_t len = msg.commands.len;
  for (uint8_t i = 0; i < len; ++i) {
    updateColor(msg.commands.data[i].light_id, msg.commands.data[i].color);
  }
}


namespace {
  void ledThread(void *arg)
  {
    (void)arg;
    chRegSetThreadName("led2812");
    systime_t next = chVTGetSystemTimeX();
    while(true) {
      for (size_t i = 0; i < ledCount; ++i) {
	(*ledStrip)[i].setRGB(desiredColors[i]);
      }
      ledStrip->emitFrame();
      next += kFramePeriod;
      chThdSleepUntil(next);
    }
  }
}

#endif // USE_LED2812_ROLE
