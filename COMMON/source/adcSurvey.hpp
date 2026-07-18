/**
 * @file adcSurvey.hpp
 * @brief ADC sampling and conversion API for board voltages and temperature.
 */
#pragma once

#include <cstdint>

namespace Adc {
  /** @brief ADC1 ownership mode selected during board startup. */
  enum class Mode : uint8_t {
    Continuous,
    OnDemand,
  };

  /** @brief Callback signature for ADC updates. */
  using Callback_t = void  (float psBat, float coreTemp);
  /** @brief Start ADC sampling and optional callback reporting. */
  void start(Callback_t *cb = nullptr, Mode mode = Mode::Continuous);
  /** @brief Refresh VIN, core temperature and VREF in OnDemand mode. */
  bool sampleOnce();
  /** @brief Register a callback for ADC error conditions. */
  void setErrorCB(Callback_t *cb);
  /** @brief Read raw battery voltage (uncalibrated). */
  float getPsBatRaw();
  /** @brief Read calibrated battery voltage. */
  float getPsBat();
  /** @brief Read calibrated core temperature. */
  float getCoreTemp();
  /** @brief Read measured VCC. */
  float getVcc();
}
