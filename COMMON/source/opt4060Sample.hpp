#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Channel decoding and CRC extracted unchanged from imav2026 (0e88dc7).
namespace Opt4060Sample {
  constexpr uint8_t maximumExponent = 6U;
  struct Channel {
    uint32_t adcCode;
    uint32_t mantissa;
    uint8_t counter;
    uint8_t exponent;
    bool valid;
  };

  constexpr uint16_t readBigEndian16(const uint8_t *bytes)
  {
    return static_cast<uint16_t>(
      static_cast<uint16_t>(bytes[0]) << 8U | bytes[1]);
  }

  constexpr uint8_t parity(uint32_t value)
  {
    return static_cast<uint8_t>(__builtin_parity(value));
  }

  /**
   * @brief Calculate TI's unrolled x^4+x+1 OPT4060 output CRC.
   *
   * The protected word contains the 20-bit mantissa, four exponent bits and
   * the four-bit rolling sample counter. These masks are the direct, compact
   * form of the four equations in the channel-result register description.
   */
  constexpr uint8_t crc(uint8_t exponent, uint32_t mantissa, uint8_t counter)
  {
    const uint32_t parityColumns = mantissa ^ exponent ^ counter;
    uint8_t crc = parity(parityColumns);
    crc |= static_cast<uint8_t>(
      parity(parityColumns & 0x000AAAAAU) << 1U);
    crc |= static_cast<uint8_t>(
      parity(parityColumns & 0x00088888U) << 2U);
    crc |= static_cast<uint8_t>(
      parity(mantissa & 0x00080808U) << 3U);
    return crc;
  }

  constexpr Channel decode(uint16_t msb, uint16_t lsb)
  {
    const uint8_t exponent = static_cast<uint8_t>(msb >> 12U);
    const uint32_t mantissa =
      (static_cast<uint32_t>(msb & 0x0FFFU) << 8U) |
      static_cast<uint32_t>(lsb >> 8U);
    const uint8_t counter = static_cast<uint8_t>((lsb >> 4U) & 0x0FU);
    const uint8_t receivedCrc = static_cast<uint8_t>(lsb & 0x0FU);
    const bool valid =
      (exponent <= maximumExponent) &&
      (crc(exponent, mantissa, counter) == receivedCrc);
    return {
      .adcCode = valid ? mantissa << exponent : 0U,
      .mantissa = mantissa,
      .counter = counter,
      .exponent = exponent,
      .valid = valid,
    };
  }

  static_assert(
    decode(0x3123U, 0x45A7U).adcCode ==
      (0x12345U << 3U));
  static_assert(decode(0x3123U, 0x45A7U).counter == 0x0AU);
  static_assert(decode(0x3123U, 0x45A7U).valid);
  static_assert(not decode(0x3123U, 0x45A6U).valid);
  static_assert(
    crc(7U, 0x12345U, 0x0AU) == 0x06U);
  static_assert(not decode(0x7123U, 0x45A6U).valid);


  struct Frame {
    std::array<uint32_t, 4U> counts = {};
    std::array<uint8_t, 4U> counters = {};
    bool checkOverload = false;
  };

  enum class Result { Valid, Corrupt, Stale };

  /** Reject corrupt or partly old RGBW frames before replacing the output. */
  inline Result decodeFrame(const uint8_t *bytes, Frame& output,
                            const Frame *previous = nullptr)
  {
    Frame next;
    for (size_t channel = 0U; channel < next.counts.size(); ++channel) {
      const size_t offset = channel * 4U;
      const auto sample = decode(readBigEndian16(bytes + offset),
                                 readBigEndian16(bytes + offset + 2U));
      if (not sample.valid) {
        return Result::Corrupt;
      }
      next.counts[channel] = sample.adcCode;
      next.counters[channel] = sample.counter;
      next.checkOverload = next.checkOverload ||
        sample.exponent == maximumExponent || sample.mantissa == 0xFFFFFU;
    }
    if (previous != nullptr) {
      for (size_t channel = 0U; channel < next.counters.size(); ++channel) {
        if (next.counters[channel] == previous->counters[channel]) {
          return Result::Stale;
        }
      }
    }
    output = next;
    return Result::Valid;
  }
}
