#include "opt4060Sample.hpp"
#include <array>
#include <cassert>
#include <cstdio>

int main()
{
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
  std::puts("Sensor decoding: RGBW CRC/freshness/wrap OK");
}
