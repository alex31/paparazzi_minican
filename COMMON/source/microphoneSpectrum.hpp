#pragma once

#include "spectrumEncoding.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

/** FFT working memory: allocate the object on the heap when the role starts. */
class MicrophoneSpectrum {
public:
  static constexpr size_t sampleCount = 512U;
  // Application selection count, independent of the message's 255-bin bound.
  static constexpr size_t maxBins = 10U;
  static constexpr uint8_t adcBits = 13U;
  static constexpr float sampleRateHz = 42'500'000.0f / 1771.0f;
  static constexpr float binWidthHz = sampleRateHz / sampleCount;
  static constexpr float lowerFrequencyHz = 100.0f;
  static constexpr float upperFrequencyHz = 3000.0f;
  static constexpr float fullScalePeak = 4096.0f;
  static constexpr float floorDbfs = -SpectrumEncoding::rangeDb(adcBits);

  struct Bin {
    float frequencyHz = 0.0f;
    float levelDbfs = 0.0f;
  };
  struct Result {
    std::array<Bin, maxBins> bins = {};
    size_t count = 0U;
    float clippedFraction = 0.0f;
  };

  // Copy DMA data first, then let the caller verify DMA sequence coherence.
  // No further reads of the DMA buffer take place during the FFT.
  void capture(const uint16_t *samples)
  {
    float sum = 0.0f;
    size_t clipped = 0U;
    for (size_t i = 0U; i < sampleCount; ++i) {
      const uint16_t sample = samples[i];
      real[i] = static_cast<float>(sample);
      sum += real[i];
      clipped += (sample <= 82U) || (sample >= 8108U);
    }
    mean = sum / sampleCount;
    clippedFraction = static_cast<float>(clipped) / sampleCount;
  }

  Result analyze()
  {
    constexpr float pi = 3.14159265358979323846f;
    // Periodic Hann: coherent gain is 1/2. Remove ADC bias before windowing.
    for (size_t i = 0U; i < sampleCount; ++i) {
      const float window = 0.5f - 0.5f * std::cos(2.0f * pi * i / sampleCount);
      real[i] = (real[i] - mean) * window;
      imaginary[i] = 0.0f;
    }

    // Iterative radix-2 FFT, in place, with O(N log N) work and no allocation.
    for (size_t i = 1U, j = 0U; i < sampleCount; ++i) {
      size_t bit = sampleCount >> 1U;
      for (; (j & bit) != 0U; bit >>= 1U) {
        j ^= bit;
      }
      j ^= bit;
      if (i < j) {
        std::swap(real[i], real[j]);
        std::swap(imaginary[i], imaginary[j]);
      }
    }
    for (size_t length = 2U; length <= sampleCount; length <<= 1U) {
      const float angle = -2.0f * pi / length;
      const float stepReal = std::cos(angle);
      const float stepImaginary = std::sin(angle);
      for (size_t offset = 0U; offset < sampleCount; offset += length) {
        float twiddleReal = 1.0f;
        float twiddleImaginary = 0.0f;
        for (size_t j = 0U; j < length / 2U; ++j) {
          const size_t a = offset + j;
          const size_t b = a + length / 2U;
          const float re = real[b] * twiddleReal - imaginary[b] * twiddleImaginary;
          const float im = real[b] * twiddleImaginary + imaginary[b] * twiddleReal;
          real[b] = real[a] - re;
          imaginary[b] = imaginary[a] - im;
          real[a] += re;
          imaginary[a] += im;
          const float nextReal = twiddleReal * stepReal - twiddleImaginary * stepImaginary;
          twiddleImaginary = twiddleReal * stepImaginary + twiddleImaginary * stepReal;
          twiddleReal = nextReal;
        }
      }
    }

    Result result;
    result.clippedFraction = clippedFraction;
    // Positive frequencies only, with Hann coherent-gain correction.
    // 0 dBFS = a bin-centred sine with peak amplitude 4096 ADC counts.
    constexpr float scale = 4.0f / (sampleCount * fullScalePeak);
    for (size_t bin = 1U; bin < sampleCount / 2U; ++bin) {
      const float frequency = bin * binWidthHz;
      if (frequency < lowerFrequencyHz || frequency > upperFrequencyHz) {
        continue;
      }
      const float power = (real[bin] * real[bin] + imaginary[bin] * imaginary[bin]) *
                          scale * scale;
      if (power <= 1.0f / (fullScalePeak * fullScalePeak)) {
        // A bin at/below one ADC count peak would saturate to level code zero.
        continue;
      }
      // Keep powers during ranking; compute logarithms only for retained bins.
      size_t position = 0U;
      while (position < result.count && result.bins[position].levelDbfs >= power) {
        ++position;
      }
      if (position == maxBins) {
        continue;
      }
      result.count = std::min(result.count + 1U, maxBins);
      for (size_t i = result.count - 1U; i > position; --i) {
        result.bins[i] = result.bins[i - 1U];
      }
      result.bins[position] = {frequency, power};
    }
    for (size_t i = 0U; i < result.count; ++i) {
      result.bins[i].levelDbfs = 10.0f * std::log10(result.bins[i].levelDbfs);
    }
    return result;
  }

private:
  std::array<float, sampleCount> real;
  std::array<float, sampleCount> imaginary;
  float mean = 0.0f;
  float clippedFraction = 0.0f;
};
