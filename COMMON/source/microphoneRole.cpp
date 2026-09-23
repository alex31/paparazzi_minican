/** @file Microphone acquisition extracted from imav2026, without beacon DSP. */
#include "roleConf.h"
#if USE_MICROPHONE_ROLE

#include "microphoneRole.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "sensorTelemetry.hpp"
#include <limits>

namespace {
  constexpr size_t bufferDepth = 1024U;
  constexpr size_t halfDepth = bufferDepth / 2U;
  constexpr eventmask_t readyEvent = EVENT_MASK(0);
  constexpr eventmask_t errorEvent = EVENT_MASK(1);
  constexpr eventmask_t audioEvents = readyEvent | errorEvent;
  // 42.5 MHz / 1771 = 23997.741 Hz, as in the original acquisition.
  constexpr gptcnt_t timerInterval = 1771U;
  constexpr GPTConfig timerConfig = {
    .frequency = 42'500'000U,
    .callback = nullptr,
    .cr2 = TIM_CR2_MMS_1,
    .dier = 0U,
  };
}

struct MicrophoneDmaState {
  adcsample_t samples[bufferDepth];
  volatile uint32_t sequence = 0U;
  volatile adcerror_t errors = 0U;
  const ADCConversionGroup *group = nullptr;
};
static_assert(sizeof(adcsample_t) == sizeof(uint16_t));

DeviceStatus MicrophoneRole::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  return DeviceStatus(DeviceStatus::MICROPHONE);
}

DeviceStatus MicrophoneRole::start(UAVCAN::Node& node)
{
  m_node = &node;
  using HR = HWResource;
  if ((ADCD2.state != ADC_STOP) ||
      not boardResource.tryAcquire(HR::PA04, HR::ADC_2, HR::TIM_6)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                        std::to_underlying(HR::ADC_2));
  }
  const auto fail = [this](DeviceStatus status) {
    free_dma(audio);
    audio = nullptr;
    boardResource.release(HR::PA04, HR::ADC_2, HR::TIM_6);
    return status;
  };
  DeviceStatus status(DeviceStatus::MICROPHONE);
  audio = try_new_dma<MicrophoneDmaState>(DeviceStatus::MICROPHONE, status);
  if (not status) {
    return fail(status);
  }

  static const ADCConversionGroup group = {
    .circular = true,
    .num_channels = 1U,
    .end_cb = &Trampoline<&MicrophoneRole::dmaCallback>::fn,
    .error_cb = &Trampoline<&MicrophoneRole::errorCallback>::fn,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_EXTSEL_SRC(13U) | ADC_CFGR_EXTEN_RISING,
    // Four 12-bit samples, shifted once: 13-bit output.
    .cfgr2 = ADC_CFGR2_ROVSE | (1U << ADC_CFGR2_OVSR_Pos) |
             (1U << ADC_CFGR2_OVSS_Pos),
    .tr1 = ADC_TR_DISABLED,
    .tr2 = ADC_TR_DISABLED,
    .tr3 = ADC_TR_DISABLED,
    .awd2cr = 0U,
    .awd3cr = 0U,
    .smpr = {0U, ADC_SMPR2_SMP_AN17(ADC_SMPR_SMP_47P5)},
    .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN17), 0U, 0U, 0U},
  };
  audio->group = &group;
  publishPeriod = TIME_MS2I(static_cast<uint32_t>(
    param_cget<"role.adc.microphone.period_ms">()));

  // ADCv3 asserts on stream exhaustion; report a regular startup error first.
  const stm32_dma_stream_t *probe = dmaStreamAlloc(
    STM32_ADC_ADC2_DMA_STREAM, STM32_ADC_ADC2_DMA_IRQ_PRIORITY, nullptr, nullptr);
  if (probe == nullptr) {
    return fail(DeviceStatus(DeviceStatus::MICROPHONE, DeviceStatus::DMA_UNAVAILABLE));
  }
  dmaStreamFree(probe);
  if (adcStart(&ADCD2, nullptr) != MSG_OK) {
    return fail(DeviceStatus(DeviceStatus::MICROPHONE, DeviceStatus::NOT_RESPONDING));
  }
  if (gptStart(&GPTD6, &timerConfig) != MSG_OK) {
    adcStop(&ADCD2);
    return fail(DeviceStatus(DeviceStatus::MICROPHONE, DeviceStatus::NOT_RESPONDING));
  }
  worker = chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(1536U),
    "microphone", NORMALPRIO, &Trampoline<&MicrophoneRole::run>::fn, this);
  if (worker == nullptr) {
    gptStop(&GPTD6);
    adcStop(&ADCD2);
    return fail(DeviceStatus(DeviceStatus::MICROPHONE, DeviceStatus::HEAP_FULL));
  }
  palSetLineMode(LINE_SPI_PERIPH_CS, PAL_MODE_INPUT_ANALOG);
  startAcquisition();
  return DeviceStatus(DeviceStatus::MICROPHONE);
}

void MicrophoneRole::startAcquisition()
{
  chSysLock();
  audio->sequence = 0U;
  audio->errors = 0U;
  chSysUnlock();
  adcStartConversion(&ADCD2, audio->group, audio->samples, bufferDepth);
  gptStartContinuous(&GPTD6, timerInterval);
}

void MicrophoneRole::stopAcquisition()
{
  gptStopTimer(&GPTD6);
  adcStopConversion(&ADCD2);
}

void MicrophoneRole::dmaCallback(ADCDriver *)
{
  chSysLockFromISR();
  ++audio->sequence;
  if (worker != nullptr) {
    chEvtSignalI(worker, readyEvent);
  }
  chSysUnlockFromISR();
}

void MicrophoneRole::errorCallback(ADCDriver *, adcerror_t error)
{
  chSysLockFromISR();
  audio->errors |= error;
  if (worker != nullptr) {
    chEvtSignalI(worker, errorEvent);
  }
  chSysUnlockFromISR();
}

void MicrophoneRole::publish(const MicrophoneStatistics& statistics, bool valid)
{
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  publishSensorValue(*m_node, "mic.ok", valid ? 1.0f : 0.0f);
  publishSensorValue(*m_node, "mic.dc", valid ? statistics.mean : invalid);
  publishSensorValue(*m_node, "mic.rms", valid ? statistics.rms : invalid);
  publishSensorValue(*m_node, "mic.pp", valid ? statistics.peakToPeak : invalid);
  publishSensorValue(*m_node, "mic.clip", valid ? statistics.clippedFraction : invalid);
  publishSensorValue(*m_node, "mic.drop", static_cast<float>(droppedBlocks));
  publishSensorValue(*m_node, "mic.rst", static_cast<float>(restarts));
}

void MicrophoneRole::run(void *)
{
  uint32_t lastSequence = 0U;
  bool discardFirst = true;
  systime_t lastBlock = chVTGetSystemTimeX();
  systime_t lastPublish = lastBlock;
  while (true) {
    const eventmask_t events = chEvtWaitAnyTimeout(audioEvents, TIME_MS2I(100U));
    chSysLock();
    const uint32_t sequence = audio->sequence;
    const adcerror_t errors = audio->errors;
    chSysUnlock();
    const systime_t now = chVTGetSystemTimeX();
    const uint32_t pending = sequence - lastSequence; // handles counter wrap
    if ((errors != 0U) || (events == 0U) ||
        ((pending == 0U) && chTimeDiffX(lastBlock, now) >= TIME_MS2I(100U))) {
      stopAcquisition();
      ++restarts;
      ++droppedBlocks;
      publish({}, false);
      lastPublish = now;
      chThdSleepMilliseconds(10U);
      (void) chEvtGetAndClearEvents(audioEvents);
      startAcquisition();
      lastSequence = 0U;
      discardFirst = true;
      lastBlock = chVTGetSystemTimeX();
      continue;
    }
    if (pending == 0U) {
      continue;
    }
    droppedBlocks += pending - 1U;
    lastSequence = sequence;
    lastBlock = now;
    // Reject the first DMA half after every restart (ADC startup errata).
    if (discardFirst) {
      discardFirst = false;
      continue;
    }
    const size_t offset = ((sequence - 1U) & 1U) * halfDepth;
    const auto statistics = microphoneStatistics(audio->samples + offset, halfDepth);
    chSysLock();
    const bool coherent = (audio->sequence == sequence) && (audio->errors == 0U);
    chSysUnlock();
    if (not coherent) {
      // DMA may have reused this half while the worker was preempted.
      ++droppedBlocks;
      if (chTimeDiffX(lastPublish, now) >= publishPeriod) {
        lastPublish = now;
        publish({}, false);
      }
      continue;
    }
    if (chTimeDiffX(lastPublish, now) >= publishPeriod) {
      lastPublish = now;
      publish(statistics, true);
    }
  }
}
#endif
