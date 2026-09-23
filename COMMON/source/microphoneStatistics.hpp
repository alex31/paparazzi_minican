#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

struct MicrophoneStatistics {
  float mean = 0.0f;
  float rms = 0.0f;
  float peakToPeak = 0.0f;
  float clippedFraction = 0.0f;
};

/** @brief DC offset and AC amplitude in 13-bit ADC counts, not calibrated SPL. */
inline MicrophoneStatistics microphoneStatistics(const uint16_t *samples,
                                                 size_t count)
{
  MicrophoneStatistics result;
  if (count == 0U) {
    return result;
  }
  uint32_t sum = 0U;
  uint16_t minimum = UINT16_MAX;
  uint16_t maximum = 0U;
  size_t clipped = 0U;
  for (size_t i = 0U; i < count; ++i) {
    const uint16_t sample = samples[i];
    sum += sample;
    minimum = std::min(minimum, sample);
    maximum = std::max(maximum, sample);
    // Same near-rail thresholds as the IM68A130 acquisition on imav2026.
    clipped += (sample <= 82U) || (sample >= 8108U);
  }
  result.mean = static_cast<float>(sum) / static_cast<float>(count);
  float squares = 0.0f;
  for (size_t i = 0U; i < count; ++i) {
    const float ac = static_cast<float>(samples[i]) - result.mean;
    squares += ac * ac;
  }
  result.rms = std::sqrt(squares / static_cast<float>(count));
  result.peakToPeak = static_cast<float>(maximum - minimum);
  result.clippedFraction = static_cast<float>(clipped) / static_cast<float>(count);
  return result;
}
