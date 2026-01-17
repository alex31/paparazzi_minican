/**
 * @file dipSwitch.hpp
 * @brief Template wrapper for reading a DIP switch connected to GPIO lines.
 */
#pragma once
#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include <array>

/**
 * @brief Read a DIP switch wired to arbitrary GPIO lines.
 *
 * The line list is stored at runtime because the platform GPIO pointer
 * values cannot be constexpr.
 */
template<size_t SZ>
class DipSwitch {
public:
  /** @brief Construct from an array of GPIO lines. */
  uint32_t read() const;
  DipSwitch(const std::array<ioline_t, SZ> _lines) : lines(_lines) {};
private:
  const std::array<ioline_t, SZ> lines;
};

template<size_t SZ>
/** @brief Read the switch state as a bitmask. */
uint32_t DipSwitch<SZ>::read() const
{
  uint32_t mask = 0;
  for(size_t i = 0; i < SZ; i++) {
      mask |= palReadLine(lines[i]) << i;
  }
  return mask;
}
