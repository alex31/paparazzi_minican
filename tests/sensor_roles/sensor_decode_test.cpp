#include "microphoneStatistics.hpp"
#include "opt4060Sample.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
  std::array<uint16_t, 512> audio;
  audio.fill(4096U);
  auto metrics = microphoneStatistics(audio.data(), audio.size());
  assert(metrics.mean == 4096.0f && metrics.rms == 0.0f);
  assert(metrics.peakToPeak == 0.0f && metrics.clippedFraction == 0.0f);
  for (size_t i = 0; i < audio.size(); ++i) {
    audio[i] = (i % 2U) == 0U ? 3096U : 5096U;
  }
  metrics = microphoneStatistics(audio.data(), audio.size());
  assert(metrics.mean == 4096.0f && metrics.rms == 1000.0f);
  assert(metrics.peakToPeak == 2000.0f && metrics.clippedFraction == 0.0f);
  const uint16_t fractional[] = {100U, 101U};
  metrics = microphoneStatistics(fractional, 2U);
  assert(metrics.mean == 100.5f && metrics.rms == 0.5f);
  const uint16_t rails[] = {0U, 82U, 83U, 8107U, 8108U, 8191U};
  metrics = microphoneStatistics(rails, 6U);
  assert(std::abs(metrics.clippedFraction - 4.0f / 6.0f) < 1e-6f);
  assert(metrics.peakToPeak == 8191.0f);
  assert(microphoneStatistics(nullptr, 0U).rms == 0.0f);

  using namespace Opt4060Sample;
  // Fixed register values used in the original driver's CRC regression check.
  std::array<uint8_t, 16> bytes = {
    0x31, 0x23, 0x45, 0xA7, 0x31, 0x23, 0x45, 0xA7,
    0x31, 0x23, 0x45, 0xA7, 0x31, 0x23, 0x45, 0xA7,
  };
  Frame frame;
  assert(decodeFrame(bytes.data(), frame) == Result::Valid);
  for (size_t i = 0; i < 4U; ++i) {
    assert(frame.counts[i] == 596520U && frame.counters[i] == 10U);
  }
  assert(not frame.checkOverload);
  const Frame old = frame;
  assert(decodeFrame(bytes.data(), frame, &old) == Result::Stale);

  // Every single-bit corruption, including CRC/counter/exponent, is rejected.
  for (size_t bit = 0; bit < 128U; ++bit) {
    auto damaged = bytes;
    damaged[bit / 8U] ^= static_cast<uint8_t>(1U << (bit % 8U));
    assert(decodeFrame(damaged.data(), frame) == Result::Corrupt);
    assert(frame.counts == old.counts && frame.counters == old.counters);
  }
  for (size_t channel = 0; channel < 3U; ++channel) {
    bytes[channel * 4U + 3U] = 0xB6; // New counter=11, valid CRC.
  }
  // A valid CRC on each channel is insufficient if one channel is still old.
  assert(decodeFrame(bytes.data(), frame, &old) == Result::Stale);
  assert(frame.counters == old.counters);
  bytes[15] = 0xB6;
  assert(decodeFrame(bytes.data(), frame, &old) == Result::Valid);
  for (size_t channel = 0; channel < 4U; ++channel) {
    bytes[channel * 4U + 3U] = 0xF7;
  }
  assert(decodeFrame(bytes.data(), frame) == Result::Valid);
  const Frame beforeWrap = frame;
  for (size_t channel = 0; channel < 4U; ++channel) {
    bytes[channel * 4U + 3U] = 0x03;
  }
  assert(decodeFrame(bytes.data(), frame, &beforeWrap) == Result::Valid);
  assert(frame.counters[0] == 0U);
  // The CRC can be correct with an unsupported exponent: still reject it.
  assert(not decode(0x7123U, 0x45A6U).valid);
  bytes[0] = 0x60; bytes[1] = 0; bytes[2] = 0; bytes[3] = 2;
  assert(decodeFrame(bytes.data(), frame) == Result::Valid);
  assert(frame.checkOverload);
  std::puts("Sensor decoding: audio amplitude/clipping, RGBW CRC/freshness/wrap OK");
}
