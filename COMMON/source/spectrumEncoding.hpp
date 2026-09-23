#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace SpectrumEncoding {
  constexpr float dbPerBit = 6.020599913f;
  constexpr float rangeDb(uint8_t adcBits)
  {
    return adcBits >= 2U && adcBits <= 32U ? dbPerBit * (adcBits - 1U) : 0.0f;
  }

  // Log amplitude from one ADC count peak to full-scale peak. Both endpoints
  // saturate; this nominal code range makes no claim about ADC noise/ENOB.
  inline uint16_t encodeLevel(float dbfs, uint8_t adcBits)
  {
    const float range = rangeDb(adcBits);
    if (range == 0.0f || std::isnan(dbfs) || dbfs <= -range) {
      return 0U;
    }
    if (dbfs >= 0.0f) {
      return UINT16_MAX;
    }
    return static_cast<uint16_t>(std::lround(65535.0f * (1.0f + dbfs / range)));
  }

  constexpr float decodeLevel(uint16_t level, uint8_t adcBits)
  {
    return rangeDb(adcBits) * (static_cast<float>(level) / 65535.0f - 1.0f);
  }

  inline uint16_t encodeFrequency(float frequencyHz)
  {
    if (std::isnan(frequencyHz) || frequencyHz <= 0.0f) {
      return 0U;
    }
    return static_cast<uint16_t>(std::lround(std::min(frequencyHz, 65535.0f)));
  }
}
