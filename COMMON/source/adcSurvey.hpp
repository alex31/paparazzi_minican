/**
 * @file adcSurvey.hpp
 * @brief ADC sampling and conversion API for board voltages and temperature.
 */
#pragma once

namespace Adc {
  /** @brief Callback signature for ADC updates. */
  using Callback_t = void  (float psBat, float coreTemp);
  /** @brief Start ADC sampling and optional callback reporting. */
  void start(Callback_t *cb = nullptr);
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
