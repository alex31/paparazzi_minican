#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>

namespace Opt4060Timing {
  constexpr std::array conversionTimesUs = {
    600U, 1000U, 1800U, 3400U, 6500U, 12'700U,
    25'000U, 50'000U, 100'000U, 200'000U, 400'000U, 800'000U,
  };

  enum class Error : uint16_t { Frequency = 1U, ScanBelowPublish, ConversionBudget };

  struct Settings {
    uint32_t scanIntervalUs;
    uint32_t publishIntervalUs;
    uint32_t conversionUs;
    uint8_t conversionIndex;
    bool onChange;

    // Auto-range, quick wake-up, forced auto-range one shot, active-low INT.
    [[nodiscard]] constexpr uint16_t triggerConfig() const {
      return static_cast<uint16_t>(0xB018U | (conversionIndex << 6U));
    }
  };

  /** Longest conversion fitting one scan, reserving bus and scheduling time. */
  [[nodiscard]] constexpr std::expected<Settings, Error>
  configure(uint32_t publishHz, uint32_t scanHz, uint32_t i2cKhz)
  {
    if (publishHz == 0U || publishHz > 200U || scanHz > 200U ||
        (i2cKhz != 100U && i2cKhz != 400U)) {
      return std::unexpected(Error::Frequency);
    }
    if (scanHz == 0U) {
      scanHz = publishHz;
    }
    if (scanHz < publishHz) {
      return std::unexpected(Error::ScanBelowPublish);
    }
    // Includes trigger/completion/status/RGBW I2C transfers, the forced
    // auto-range startup (~500 us), and RTOS wake-up margin. Bus contention
    // can still delay a scan; the worker skips missed deadlines, never bursts.
    const uint32_t marginUs = i2cKhz == 100U ? 4000U : 2000U;
    for (size_t i = conversionTimesUs.size(); i-- > 0U;) {
      if (4U * conversionTimesUs[i] + marginUs <= 1'000'000U / scanHz) {
        return Settings{
          .scanIntervalUs = (1'000'000U + scanHz - 1U) / scanHz,
          .publishIntervalUs = (1'000'000U + publishHz - 1U) / publishHz,
          .conversionUs = conversionTimesUs[i],
          .conversionIndex = static_cast<uint8_t>(i),
          .onChange = scanHz > publishHz,
        };
      }
    }
    return std::unexpected(Error::ConversionBudget);
  }
}
