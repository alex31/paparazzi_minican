#include "roleConf.h"

#if USE_VOLTMETER_ROLE

#include "voltmeterRole.hpp"

#include <algorithm>
#include <cmath>

#include "UAVCAN/persistantParam.hpp"
#include "adcSurvey.hpp"
#include "hardwareConf.hpp"
#include "led2812.hpp"
#include "resourceManager.hpp"

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT3_NX(st1, st2, st3) st1 ## st2 ## st3
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)
#define CONCAT3(st1, st2, st3) CONCAT3_NX(st1, st2, st3)

namespace {
  constexpr size_t kLedCount = 8;
  constexpr systime_t kFramePeriod = TIME_MS2I(50);
  constexpr systime_t kEmptyFlashPeriod = TIME_MS2I(500);
  constexpr systime_t kGpsTimeout = TIME_S2I(2);
  constexpr float kSpeedHysteresisMps = 0.5f;
  constexpr float kCellEmptyVoltage = 3.4f;
  constexpr float kCellFullVoltage = 4.2f;

  static constexpr PWMDriver &ledPwm = LedStripPWMD;
  static constexpr uint32_t dmaMux = CONCAT3(STM32_DMAMUX1_TIM, LED2812_TIM, _UP);
  static constexpr LedTiming ledTiming = getClockByTimer(&ledPwm);
  using Led_t = Led2812<uint16_t, ledTiming.t0h, ledTiming.t1h>;

  Led2812Strip<kLedCount, Led_t> *ledStrip = nullptr;

  THD_WORKING_AREA(waMinican, 512);

  uint8_t getCells()
  {
    const auto cells = static_cast<int>(param_cget<"role.voltmeter.cells">());
    return static_cast<uint8_t>(std::clamp(cells, 2, 6));
  }

  float getBrightness()
  {
    return std::clamp(param_cget<"role.voltmeter.brightness">(), 0.0f, 1.0f);
  }

  float getSpeedOffThresholdMps()
  {
    const float v = param_cget<"role.voltmeter.gps_speed_off_mps">();
    return std::clamp(v, 0.0f, 100.0f);
  }

  float lipoSocFromCellVoltage(float cellV)
  {
    struct Point {
      float v;
      float soc;
    };

    // Typical LiPo OCV-ish curve anchors; good enough for "on tarmac, motors off".
    static constexpr Point curve[] = {
      {3.40f, 0.00f},
      {3.50f, 0.02f},
      {3.60f, 0.05f},
      {3.70f, 0.15f},
      {3.75f, 0.25f},
      {3.80f, 0.40f},
      {3.85f, 0.50f},
      {3.90f, 0.60f},
      {3.95f, 0.70f},
      {4.00f, 0.80f},
      {4.10f, 0.90f},
      {4.15f, 0.95f},
      {4.20f, 1.00f},
    };

    if (cellV <= curve[0].v) {
      return curve[0].soc;
    }
    if (cellV >= curve[(sizeof curve / sizeof curve[0]) - 1].v) {
      return curve[(sizeof curve / sizeof curve[0]) - 1].soc;
    }

    for (size_t i = 1; i < (sizeof curve / sizeof curve[0]); ++i) {
      if (cellV <= curve[i].v) {
        const float v0 = curve[i - 1].v;
        const float v1 = curve[i].v;
        const float soc0 = curve[i - 1].soc;
        const float soc1 = curve[i].soc;
        const float t = (cellV - v0) / (v1 - v0);
        return soc0 + t * (soc1 - soc0);
      }
    }

    return 0.0f;
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

void VoltmeterRole::processFix2(CanardRxTransfer*, const uavcan_equipment_gnss_Fix2& msg)
{
  if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX) {
    chSysLock();
    m_gpsSpeedValid = false;
    chSysUnlock();
    return;
  }

  const float vn = msg.ned_velocity[0];
  const float ve = msg.ned_velocity[1];
  const float groundSpeedMps = std::sqrt(vn * vn + ve * ve);
  if (!std::isfinite(groundSpeedMps)) {
    return;
  }

  chSysLock();
  m_groundSpeedMps = groundSpeedMps;
  m_lastGpsUpdate = chVTGetSystemTimeX();
  m_gpsSpeedValid = true;
  chSysUnlock();
}

DeviceStatus VoltmeterRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&VoltmeterRole::processFix2>::fn>();
  return DeviceStatus(DeviceStatus::VOLTMETER_ROLE);
}

DeviceStatus VoltmeterRole::start(UAVCAN::Node& /*node*/)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::VOLTMETER_ROLE);

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

  static Led2812Strip<kLedCount, Led_t> strip(&ledPwm, ledTiming,
                                              STM32_DMA_STREAM_ID_ANY,
                                              dmaMux,
                                              static_cast<TimerChannel>(LED2812_TIM_CH - 1U),
                                              kLedCount);
  ledStrip = &strip;

  chThdCreateStatic(waMinican, sizeof(waMinican), NORMALPRIO,
                    &VoltmeterRole::voltmeterThread, this);
  return status;
}

void VoltmeterRole::voltmeterThread(void* arg)
{
  auto* self = static_cast<VoltmeterRole*>(arg);
  chRegSetThreadName("voltmeter");

  systime_t next = chVTGetSystemTimeX();
  systime_t lastFlash = next;
  bool flashOn = true;

  bool blanked = false;
  bool blankFrameSent = false;

  while (true) {
    const float speedOffMps = getSpeedOffThresholdMps();
    const systime_t now = chVTGetSystemTimeX();

    bool gpsValid = false;
    float groundSpeedMps = 0.0f;
    systime_t lastGpsUpdate = 0;
    {
      chSysLock();
      gpsValid = self->m_gpsSpeedValid;
      groundSpeedMps = self->m_groundSpeedMps;
      lastGpsUpdate = self->m_lastGpsUpdate;
      chSysUnlock();
    }

    if (gpsValid && (chTimeDiffX(lastGpsUpdate, now) > kGpsTimeout)) {
      gpsValid = false;
    }

    if ((speedOffMps > 0.0f) && gpsValid) {
      if (!blanked && (groundSpeedMps > speedOffMps)) {
        blanked = true;
      } else if (blanked && (groundSpeedMps < (speedOffMps - kSpeedHysteresisMps))) {
        blanked = false;
      }
    } else {
      blanked = false;
    }

    if (blanked) {
      if (!blankFrameSent) {
        setAll(RGB{0, 0, 0});
        ledStrip->emitFrame();
        blankFrameSent = true;
      }
      next += kFramePeriod;
      chThdSleepUntil(next);
      continue;
    }
    blankFrameSent = false;

    const float voltage = Adc::getPsBat();
    const uint8_t cells = getCells();
    const float cellVoltage = voltage / static_cast<float>(cells);
    const float brightness = getBrightness();

    if (cellVoltage >= kCellFullVoltage) {
      setAll(hsv2rgb(HSV{0.333f, 1.0f, brightness}));
    } else if (cellVoltage <= kCellEmptyVoltage) {
      if (chTimeDiffX(lastFlash, now) >= kEmptyFlashPeriod) {
        lastFlash = now;
        flashOn = !flashOn;
      }
      setAll(flashOn ? hsv2rgb(HSV{0.0f, 1.0f, brightness}) : RGB{0, 0, 0});
    } else {
      const float pct = std::clamp(lipoSocFromCellVoltage(cellVoltage), 0.0f, 1.0f);
      const float scaled = pct * static_cast<float>(kLedCount);
      const size_t litCount =
        std::clamp<size_t>(static_cast<size_t>(std::ceil(scaled)), 1U, kLedCount);
      setGauge(litCount, hsv2rgb(pickColor(pct, brightness)));
    }

    ledStrip->emitFrame();
    next += kFramePeriod;
    chThdSleepUntil(next);
  }
}

#endif // USE_VOLTMETER_ROLE
