#include "roleConf.h"

#if USE_VOLTMETER_ROLE

#include "roleVoltmeter.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "UAVCAN/persistantParam.hpp"
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"
#include "led2812.hpp"
#include "ressourceManager.hpp"

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT3_NX(st1, st2, st3) st1 ## st2 ## st3
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)
#define CONCAT3(st1, st2, st3) CONCAT3_NX(st1, st2, st3)

namespace {
  constexpr size_t kLedCount = 8;
  constexpr systime_t kFramePeriod = TIME_MS2I(50);
  constexpr systime_t kEmptyFlashPeriod = TIME_MS2I(500);

  static constexpr PWMDriver &ledPwm = LedStripPWMD;
  static constexpr uint32_t dmaMux = CONCAT3(STM32_DMAMUX1_TIM, LED2812_TIM, _UP);
  static constexpr LedTiming ledTiming = getClockByTimer(&ledPwm);
  using Led_t = Led2812<uint16_t, ledTiming.t0h, ledTiming.t1h>;

  Led2812Strip<kLedCount, Led_t> *ledStrip = nullptr;

  THD_WORKING_AREA(waMinican, 512);
  void voltmeterThread(void *arg);

  float getMinVoltage()
  {
    return param_cget<"role.voltmeter.min_voltage">();
  }

  float getMaxVoltage()
  {
    return param_cget<"role.voltmeter.max_voltage">();
  }

  float getBrightness()
  {
    return std::clamp(param_cget<"role.voltmeter.brightness">(), 0.0f, 1.0f);
  }

  HSV pickColor(float pct, float brightness)
  {
    constexpr float kHueRed = 0.0f;
    constexpr float kHueYellow = 0.166f;
    constexpr float kHueGreen = 0.333f;

    float hue = kHueGreen;
    if (pct <= 0.2f) {
      hue = kHueRed;
    } else if (pct <= 0.4f) {
      hue = kHueYellow;
    }
    return HSV{hue, 1.0f, brightness};
  }

  void setAll(const RGB &rgb)
  {
    for (size_t i = 0; i < kLedCount; ++i) {
      (*ledStrip)[i].setRGB(rgb);
    }
  }

  void setGauge(size_t litCount, const RGB &rgb)
  {
    litCount = std::min(litCount, kLedCount);
    for (size_t i = 0; i < kLedCount; ++i) {
      (*ledStrip)[i].setRGB(i < litCount ? rgb : RGB{0, 0, 0});
    }
  }
}

DeviceStatus RoleVoltmeter::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  return DeviceStatus(DeviceStatus::VOLTMETER_ROLE);
}

DeviceStatus RoleVoltmeter::start(UAVCAN::Node& /*node*/)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::VOLTMETER_ROLE);

  const float minV = getMinVoltage();
  const float maxV = getMaxVoltage();
  if (!(maxV > minV)) {
    return DeviceStatus(DeviceStatus::VOLTMETER_ROLE, DeviceStatus::INVALID_PARAM);
  }

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

  static Led2812Strip<kLedCount, Led_t> strip(&ledPwm, ledTiming,
                                              STM32_DMA_STREAM_ID_ANY,
                                              dmaMux,
                                              static_cast<TimerChannel>(LED2812_TIM_CH - 1U),
                                              kLedCount);
  ledStrip = &strip;

  chThdCreateStatic(waMinican, sizeof(waMinican), NORMALPRIO,
                    &voltmeterThread, nullptr);
  return status;
}

namespace {
  void voltmeterThread(void *arg)
  {
    (void)arg;
    chRegSetThreadName("voltmeter");

    systime_t next = chVTGetSystemTimeX();
    systime_t lastFlash = next;
    bool flashOn = true;

    while (true) {
      const float voltage = Adc::getPsBat();
      const float minV = getMinVoltage();
      const float maxV = getMaxVoltage();
      const float brightness = getBrightness();

      if (maxV <= minV) {
        setAll(RGB{0, 0, 0});
      } else if (voltage >= maxV) {
        setAll(hsv2rgb(HSV{0.333f, 1.0f, brightness}));
      } else if (voltage <= minV) {
        const systime_t now = chVTGetSystemTimeX();
        if (chTimeDiffX(lastFlash, now) >= kEmptyFlashPeriod) {
          lastFlash = now;
          flashOn = !flashOn;
        }
        setAll(flashOn ? hsv2rgb(HSV{0.0f, 1.0f, brightness}) : RGB{0, 0, 0});
      } else {
        const float pct = std::clamp((voltage - minV) / (maxV - minV), 0.0f, 1.0f);
        const long litRounded = std::lround(pct * static_cast<float>(kLedCount));
        const size_t litCount = pct > 0.0f ? std::max<size_t>(1U, static_cast<size_t>(std::clamp(litRounded, 0L, static_cast<long>(kLedCount))))
                                           : 0U;
        setGauge(litCount, hsv2rgb(pickColor(pct, brightness)));
      }

      ledStrip->emitFrame();
      next += kFramePeriod;
      chThdSleepUntil(next);
    }
  }
}

#endif // USE_VOLTMETER_ROLE
