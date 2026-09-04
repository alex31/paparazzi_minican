/**
 * @file imavLightRange.hpp
 * @brief Downward light-flash and ground-range acquisition for the IMAV role.
 */
#pragma once

#include "imavVl53l4cxPort.h"
#include "UAVCAN/pubSub.hpp"

#include <array>

extern "C" {
#include "vl53l4cx.h"
}

/** @brief Coherent telemetry snapshot shared with the audio worker. */
struct ImavLightRangeSnapshot {
  bool lightAvailable = false;
  bool rangeAvailable = false;
  bool rangeValid = false;
  uint8_t lightAddress = 0x44U;
  uint16_t lightDeviceId = 0U;
  uint32_t lightRed = 0U;
  uint32_t lightGreen = 0U;
  uint32_t lightBlue = 0U;
  uint32_t lightWide = 0U;
  float lightRedRatio = 0.0f;
  float lightRelativeAc = 0.0f;
  float lightInstantScore = 0.0f;
  float lightCadenceHz = 0.0f;
  float lightCadenceScore = 0.0f;
  float lightFastScore = 0.0f;
  float lightPulseStrength = 0.0f;
  float lightHighDurationMs = 0.0f;
  float lightLowDurationMs = 0.0f;
  float lightTemporalShapeScore = 0.0f;
  uint8_t lightPattern = 0U;
  uint32_t lightSamples = 0U;
  uint32_t lightPulses = 0U;
  uint32_t lightSaturations = 0U;
  uint32_t lightReadErrors = 0U;
  uint32_t lightBusRecoveries = 0U;
  uint32_t lightGaps = 0U;
  systime_t lightLastSample = 0U;
  uint32_t rangeDeviceId = 0U;
  uint16_t rangeMm = 0U;
  float rangeSignalKcps = 0.0f;
  float rangeAmbientKcps = 0.0f;
  uint32_t rangeMeasurements = 0U;
  uint32_t rangeErrors = 0U;
  systime_t rangeLastSample = 0U;
};

/**
 * @brief Owns one OPT4060 and one VL53L4CX on the shared external I2C bus.
 *
 * Only this worker invokes the sensor APIs. The official ST VL53L4CX driver
 * remains untouched; its platform callbacks are implemented here using the
 * ChibiOS I2C API.
 */
class ImavLightRange final {
public:
  ImavLightRange(UAVCAN::Node& node, uint8_t lightAddress,
                 uint32_t rangePeriodMs, bool timeOfFlightEnabled,
                 bool beginningPatternEnabled, uint16_t lightHighMs,
                 uint16_t lightSteadyLowMs,
                 uint16_t lightBeginningLowMs);

  /** @brief Optionally configure ToF, then start light sampling. */
  void initialize();

  /** @brief Service both sensors forever with mutually exclusive acquisition. */
  [[noreturn]] void run();

  /** @brief Return a short, IRQ-coherent copy of published sensor metrics. */
  ImavLightRangeSnapshot snapshot() const;

private:
  struct LightFastSample;
  struct LightFastPatternState;

  friend uint8_t *imav_vl53l4cx_work_buffer(const void *device,
                                            uint32_t requiredSize);

  static int32_t tofBusInit();
  static int32_t tofBusDeinit();
  static int32_t tofBusWrite(uint16_t address, uint8_t *data,
                             uint16_t length);
  static int32_t tofBusRead(uint16_t address, uint8_t *data,
                            uint16_t length);
  static int32_t tofGetTick();

  bool initializeLight();
  bool startLight();
  bool stopLight();
  bool sampleLight(systime_t sampleTime);
  void processLightMeasurement(const std::array<uint32_t, 4U>& adcCodes,
                               bool overloaded, systime_t now);
  void resetLightFastSpectrum();
  void accumulateLightFastSample(const LightFastSample& sample,
                                 float direction);
  void scoreLightFastPattern(LightFastPatternState& pattern,
                             uint16_t lowMs, uint8_t identifier);
  void evaluateLightFastSpectrum();
  void updateLightFastSpectrum(const std::array<float, 4U>& scaled,
                               systime_t now);
  void updateSynchronizedLightEvent(systime_t now, bool scoreIsCurrent);
  bool readLightRegister(uint8_t reg, uint16_t& value);
  bool readLightBlock(uint8_t reg, size_t length);
  bool writeLightRegister(uint8_t reg, uint16_t value);
  msg_t lightTransfer(size_t txLength, size_t rxLength);

  bool initializeRange();
  bool measureRange();
  void publishRange(bool valid);
  msg_t tofTransfer(uint16_t address, const uint8_t *tx, size_t txLength,
                    size_t rxLength);
  void recordI2cFailure(bool rangeSensor, msg_t result);
  void publishLightScore(float score);
  void publishLightState();
  void publishLightDebugSample(bool overloaded);
  void publishAvailability();
  uint32_t nextRangeIntervalMs();

  static inline ImavLightRange *active = nullptr;

  UAVCAN::Node& node;
  uint8_t lightAddress;
  const uint32_t rangePeriodMs;
  const bool timeOfFlightEnabled;
  const bool beginningPatternEnabled;
  const uint16_t lightHighMs;
  const uint16_t lightSteadyLowMs;
  const uint16_t lightBeginningLowMs;
  VL53L4CX_Object_t rangeDevice = {};

  // This object is allocated in SRAM1, which is DMA-accessible on STM32G491.
  alignas(4) uint8_t lightTx[3] = {};
  alignas(4) uint8_t lightRx[16] = {};
  alignas(4) uint8_t tofTx[260] = {};
  alignas(4) uint8_t tofRx[260] = {};
  uint8_t pendingTofRegister[2] = {};
  uint16_t pendingTofAddress = 0U;
  bool pendingTofRead = false;

  bool lightAvailable = false;
  bool lightRunning = false;
  bool lightRestartPending = false;
  bool lightBaselineValid = false;
  bool lightNoiseValid = false;
  bool lightCountersValid = false;
  bool lightOverloadActive = false;
  bool lightFastDetected = false;
  std::array<float, 4U> lightBaseline = {};
  std::array<uint8_t, 4U> lightCounters = {};

  struct LightFastSample {
    systime_t timestamp = 0U;
    std::array<float, 3U> rgb = {};
  };
  struct LightFastBin {
    std::array<float, 3U> fundamentalReal = {};
    std::array<float, 3U> fundamentalImag = {};
    std::array<float, 3U> harmonicReal = {};
    std::array<float, 3U> harmonicImag = {};
    float fundamentalOscillatorReal = 0.0f;
    float fundamentalOscillatorImag = 0.0f;
    float harmonicOscillatorReal = 0.0f;
    float harmonicOscillatorImag = 0.0f;
  };
  struct LightFastPatternState {
    std::array<LightFastBin, 5U> bins = {};
    float score = 0.0f;
    float coherence = 0.0f;
    float frequencyHz = 0.0f;
    float harmonicRatio = 0.0f;
    float phaseAlignment = 0.0f;
    float redFraction = 0.0f;
    float harmonicShape = 0.0f;
    float fundamentalAmplitude = 0.0f;
    float synchronizationPhase = 0.0f;
    uint8_t identifier = 0U;
  };
  std::array<LightFastSample, 144U> lightFastSamples = {};
  LightFastPatternState lightFastSteadySpectrum = {};
  LightFastPatternState lightFastBeginningSpectrum = {};
  std::array<float, 3U> lightFastRgbSum = {};
  float lightFastContrastSum = 0.0f;
  float lightFastContrastSquareSum = 0.0f;
  size_t lightFastFirstSample = 0U;
  size_t lightFastSampleCount = 0U;
  uint32_t lightFastEvaluationCounter = 0U;
  systime_t lightFastLastSample = 0U;
  systime_t lightLastSynchronizedEvent = 0U;
  uint16_t lightSynchronizationPeriodMs = 0U;

  float lightNoiseFloor = 0.0f;
  float lightRedRatio = 0.0f;
  float lightRelativeAc = 0.0f;
  float lightInstantScore = 0.0f;
  float lightCadenceHz = 0.0f;
  float lightCadenceScore = 0.0f;
  float lightFastScore = 0.0f;
  float lightHighDurationMs = 0.0f;
  float lightLowDurationMs = 0.0f;
  float lightTemporalShapeScore = 0.0f;
  float lightRecentPulseStrength = 0.0f;
  uint32_t lightRed = 0U;
  uint32_t lightGreen = 0U;
  uint32_t lightBlue = 0U;
  uint32_t lightWide = 0U;
  uint32_t lightSamples = 0U;
  uint32_t lightPulses = 0U;
  uint32_t lightSaturations = 0U;
  uint32_t lightReadErrors = 0U;
  uint32_t lightBusRecoveries = 0U;
  uint32_t lightGaps = 0U;
  uint8_t lightConsecutiveErrors = 0U;
  uint8_t lightStalePolls = 0U;
  uint8_t lightPattern = 0U;
  systime_t lightLastSample = 0U;

  bool rangeAvailable = false;
  bool rangeValid = false;
  uint32_t rangeDeviceId = 0U;
  uint16_t rangeMm = 0U;
  float rangeSignalKcps = 0.0f;
  float rangeAmbientKcps = 0.0f;
  uint32_t rangeMeasurements = 0U;
  uint32_t rangeErrors = 0U;
  uint8_t rangeConsecutiveErrors = 0U;
  uint8_t rangeJitterIndex = 0U;
  systime_t rangeLastSample = 0U;

  mutable ImavLightRangeSnapshot published = {};
};
