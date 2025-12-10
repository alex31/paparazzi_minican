#include "roleConf.h"

#if USE_LED2812_ROLE

#include "rgbLedRole.hpp"

#include <algorithm>
#include <array>

#include "UAVCAN/persistantParam.hpp"
#include "led2812.hpp"
#include "ressourceManager.hpp"


namespace {
  constexpr size_t kMaxLeds = 8;
  constexpr systime_t kFramePeriod = TIME_MS2I(20);

  // Fixed hardware mapping: PB07 / TIM3_CH4
  static constexpr PWMDriver &ledPwm = PWMD3;
  static constexpr uint32_t dmaMux = STM32_DMAMUX1_TIM3_UP;
  static constexpr LedTiming ledTiming = getClockByTimer(&ledPwm);
  using Led_t = Led2812<uint16_t, ledTiming.t0h, ledTiming.t1h>;

  Led2812Strip<kMaxLeds, Led_t> *ledStrip = nullptr;
  size_t ledCount = 1;
  bool mtxInited = false;

  mutex_t colorsMtx;
  std::array<RGB, kMaxLeds> desiredColors{};

  THD_WORKING_AREA(waLedStrip, 512);
  void ledThread(void *arg);

  RGB convertRgb565(const uavcan_equipment_indication_RGB565 &c)
  {
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
    if (!mtxInited) {
      chMtxObjectInit(&colorsMtx);
      mtxInited = true;
    }
    chMtxLock(&colorsMtx);
    desiredColors[idx] = convertRgb565(color);
    chMtxUnlock(&colorsMtx);
  }
}


DeviceStatus RgbLedRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  ledCount = std::clamp<size_t>(param_cget<"role.led2812.led_number">(), 1U, kMaxLeds);
  if (!mtxInited) {
    chMtxObjectInit(&colorsMtx);
    mtxInited = true;
  }
  node.subscribeBroadcastMessages<Trampoline<&RgbLedRole::processLightsCommand>::fn>();
  return DeviceStatus::LED2812_ROLE;
}


DeviceStatus RgbLedRole::start(UAVCAN::Node& /*node*/)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::LED2812_ROLE);

  const size_t activeLedCount = std::clamp(ledCount, size_t{1}, kMaxLeds);

  if (not boardResource.tryAcquire(HR::TIM_3, HR::PB07)) {
    const auto conflict = boardResource.isAllocated(HR::TIM_3) ? HR::TIM_3 : HR::PB07;
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
					     TimerChannel::C4,
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
    std::array<RGB, kMaxLeds> colors{};
    systime_t next = chVTGetSystemTimeX();
    while(true) {
      if (mtxInited) {
	chMtxLock(&colorsMtx);
	std::copy_n(desiredColors.begin(), ledCount, colors.begin());
	chMtxUnlock(&colorsMtx);
      } else {
	colors.fill({0,0,0});
      }

      for (size_t i = 0; i < ledCount; ++i) {
	(*ledStrip)[i].setRGB(colors[i]);
      }
      ledStrip->emitFrame();
      next += kFramePeriod;
      chThdSleepUntil(next);
    }
  }
}

#endif // USE_LED2812_ROLE
