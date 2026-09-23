#include "microphoneSpectrum.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <vector>

using Spectrum = MicrophoneSpectrum;
using Samples = std::array<uint16_t, Spectrum::sampleCount>;

static Samples tone(double bin, double peak, double dc = 4096.0)
{
  Samples samples;
  for (size_t i = 0; i < samples.size(); ++i) {
    samples[i] = static_cast<uint16_t>(std::lround(dc + peak *
      std::sin(2.0 * std::numbers::pi * bin * i / samples.size())));
  }
  return samples;
}

// Independent O(N^2), double precision DFT oracle checks FFT and top-10 ranking.
static void compareDft(Spectrum& fft, const Samples& samples)
{
  double mean = 0.0;
  for (auto sample : samples) { mean += sample; }
  mean /= samples.size();
  std::vector<Spectrum::Bin> expected;
  for (size_t k = 1; k < samples.size() / 2U; ++k) {
    const float frequency = k * Spectrum::binWidthHz;
    if (frequency < 100.0f || frequency > 3000.0f) { continue; }
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < samples.size(); ++i) {
      const double phase = 2.0 * std::numbers::pi * i / samples.size();
      const double value = (samples[i] - mean) * (0.5 - 0.5 * std::cos(phase));
      re += value * std::cos(k * phase);
      im -= value * std::sin(k * phase);
    }
    const double amplitude = 4.0 * std::hypot(re, im) /
                             (samples.size() * Spectrum::fullScalePeak);
    if (amplitude > 1.0 / Spectrum::fullScalePeak) {
      expected.push_back({frequency, static_cast<float>(20.0 * std::log10(amplitude))});
    }
  }
  std::sort(expected.begin(), expected.end(), [](auto a, auto b) {
    return a.levelDbfs > b.levelDbfs;
  });
  expected.resize(std::min(expected.size(), Spectrum::maxBins));
  fft.capture(samples.data());
  const auto result = fft.analyze();
  assert(result.count == expected.size());
  for (size_t i = 0; i < result.count; ++i) {
    assert(result.bins[i].frequencyHz == expected[i].frequencyHz);
    assert(std::abs(result.bins[i].levelDbfs - expected[i].levelDbfs) < 0.01f);
  }
}

int main()
{
  auto fft = std::make_unique<Spectrum>();
  Samples silence;
  silence.fill(4096U);
  fft->capture(silence.data());
  assert(fft->analyze().count == 0U);

  auto samples = tone(21.0, 2048.0);
  fft->capture(samples.data());
  // Reusing the DMA memory after capture must not corrupt the spectrum.
  samples.fill(0U);
  const auto halfScale = fft->analyze();
  assert(halfScale.count == 3U); // Hann centre and its two neighbours.
  assert(halfScale.bins[0].frequencyHz == 21.0f * Spectrum::binWidthHz);
  assert(std::abs(halfScale.bins[0].levelDbfs + 6.0206f) < 0.002f);
  assert(halfScale.clippedFraction == 0.0f);
  samples = tone(21.0, 1024.0, 3100.0);
  fft->capture(samples.data());
  const auto quarterScale = fft->analyze();
  assert(std::abs(quarterScale.bins[0].levelDbfs - halfScale.bins[0].levelDbfs +
                  6.0206f) < 0.003f);
  samples = tone(21.0, 4094.0);
  fft->capture(samples.data());
  const auto fullScale = fft->analyze();
  assert(std::abs(fullScale.bins[0].levelDbfs) < 0.01f);
  assert(fullScale.clippedFraction > 0.0f);

  // Candidate centres include bins 3 and 64, exclude 2 and 65.
  for (double bin : {3.0, 64.0}) {
    samples = tone(bin, 1800.0);
    fft->capture(samples.data());
    assert(fft->analyze().bins[0].frequencyHz == bin * Spectrum::binWidthHz);
  }
  // Non-bin-centred tones, DC bias and out-of-band energy, including Nyquist.
  for (size_t i = 0; i < samples.size(); ++i) {
    const double phase = 2.0 * std::numbers::pi * i / samples.size();
    samples[i] = static_cast<uint16_t>(std::lround(3700.0 +
      400.0 * std::sin(12.25 * phase) + 700.0 * std::sin(37.65 * phase) +
      200.0 * std::cos(3.0 * phase) + 300.0 * std::cos(64.0 * phase) +
      1000.0 * std::sin(90.0 * phase) + 100.0 * std::cos(256.0 * phase)));
  }
  compareDft(*fft, samples);
  uint32_t random = 42U;
  for (auto& sample : samples) {
    random = random * 1664525U + 1013904223U;
    sample = 2048U + ((random >> 16U) & 4095U);
  }
  compareDft(*fft, samples);
  std::puts("Microphone spectrum: 512-point FFT vs DFT, dBFS, DC, band, top-10, clipping and copy OK");
}
