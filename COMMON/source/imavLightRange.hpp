/**
 * @file imavLightRange.hpp
 * @brief Downward light-flash and ground-range acquisition for the IMAV role.
 */
#pragma once

#include "UAVCAN/pubSub.hpp"

extern "C" {
#include "vl53l4cx.h"
}

/** @brief Coherent telemetry snapshot shared with the audio worker. */
struct ImavLightRangeSnapshot {
  bool lightAvailable = false;
  bool rangeAvailable = false;
  bool rangeValid = false;
  uint8_t lightAddress = 0x39U;
  uint8_t lightDeviceId = 0U;
  uint16_t lightRaw = 0U;
  float lightRelativeAc = 0.0f;
  float lightFlashScore = 0.0f;
  uint32_t lightSamples = 0U;
  uint32_t lightPulses = 0U;
  uint32_t lightSaturations = 0U;
  uint32_t lightFifoOverflows = 0U;
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
 * @brief Owns one TCS3410 and one VL53L4CX on the shared external I2C bus.
 *
 * Only this worker invokes the sensor APIs. The official ST VL53L4CX driver
 * remains untouched; its platform callbacks are implemented here using the
 * ChibiOS I2C API.
 */
class ImavLightRange final {
public:
  ImavLightRange(UAVCAN::Node& node, uint8_t lightAddress,
                 uint32_t rangePeriodMs);

  /** @brief Probe/configure the ToF sensor first, then start light sampling. */
  void initialize();

  /** @brief Service both sensors forever with mutually exclusive acquisition. */
  [[noreturn]] void run();

  /** @brief Return a short, IRQ-coherent copy of published sensor metrics. */
  ImavLightRangeSnapshot snapshot() const;

private:
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
  bool drainLightFifo();
  void processLightSamples(const uint8_t *bytes, size_t length);
  bool readLightRegister(uint8_t reg, uint8_t& value);
  bool readLightBlock(uint8_t reg, size_t length);
  bool writeLightRegister(uint8_t reg, uint8_t value);
  msg_t lightTransfer(size_t txLength, size_t rxLength);

  bool initializeRange();
  bool measureRange();
  void publishRange(bool valid);
  msg_t tofTransfer(uint16_t address, const uint8_t *tx, size_t txLength,
                    size_t rxLength);
  void recordI2cFailure(bool rangeSensor, msg_t result);
  void publishLightState();
  void publishAvailability();
  uint32_t nextRangeIntervalMs();

  static inline ImavLightRange *active = nullptr;

  UAVCAN::Node& node;
  uint8_t lightAddress;
  const uint32_t rangePeriodMs;
  VL53L4CX_Object_t rangeDevice = {};

  // This object is allocated in SRAM1, which is DMA-accessible on STM32G491.
  alignas(4) uint8_t lightTx[2] = {};
  alignas(4) uint8_t lightRx[128] = {};
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
  bool lightPulseActive = false;
  float lightBaseline = 0.0f;
  float lightNoiseFloor = 0.0f;
  float lightFlashHold = 0.0f;
  uint16_t lightRaw = 0U;
  float lightRelativeAc = 0.0f;
  uint32_t lightSamples = 0U;
  uint32_t lightPulses = 0U;
  uint32_t lightSaturations = 0U;
  uint32_t lightFifoOverflows = 0U;
  uint32_t lightReadErrors = 0U;
  uint32_t lightBusRecoveries = 0U;
  uint32_t lightGaps = 0U;
  uint8_t lightConsecutiveErrors = 0U;
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
