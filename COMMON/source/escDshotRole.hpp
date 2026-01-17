/**
 * @file escDshotRole.hpp
 * @brief DShot ESC role interface and channel mapping helpers.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include <array>
#include <cstdint>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "esc_dshot.h"

/**
 * @brief Translate a 4-bit channel mask into a packed index list.
 *
 * The map preserves channel order and exposes the number of enabled channels.
 */
struct DshotChannelMap {
  std::array<std::uint8_t, 4> idx{};
  std::uint8_t count{};
  std::uint8_t mask{};
  constexpr DshotChannelMap() = default;
  /** @brief Build a map from a channel mask. */
  constexpr DshotChannelMap(std::uint8_t _mask) { *this = _mask; }

  /** @brief Recompute the map from a channel mask. */
  constexpr DshotChannelMap& operator=(std::uint8_t _mask) {
    mask = _mask;
    count = 0;
    for (std::uint8_t bit = 0; bit < 4; ++bit) {
      if (mask & (1u << bit)) {
        idx[count++] = bit;
      }
    }
    return *this;
  }

  /** @brief Return the mapped channel index or 0 when out of range. */
  constexpr std::uint8_t operator[](std::size_t i) const {
    return i < count ? idx[i] : 0;
  }
};



/**
 * @brief Role providing DShot ESC command and optional telemetry support.
 */
class EscDshot : public RoleBase, public RoleCrtp<EscDshot> {
public:
  /** @brief Subscribe to the UAVCAN messages required by this role. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Acquire hardware resources and start the DShot worker thread. */
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  /** @brief Periodic worker thread for DShot output and telemetry. */
  void periodic(void *);
  /** @brief Handle incoming raw ESC command messages. */
  void processEscRawCommand(CanardRxTransfer* transfer, const uavcan_equipment_esc_RawCommand& msg);

  uint8_t mapIndex1 = 0;
  uint8_t numChannels = 1;
  DshotChannelMap channelMap{};
  uint16_t rpmFrqDiv = 0;
  sysinterval_t loopPeriod = TIME_MS2I(10);
  uint16_t throttles[4] = {};
  DSHOTDriver dshotd;
  DSHOTConfig dshotConfig;
  DshotDmaBuffer dshotdDmaBuffer;
#if DSHOT_BIDIR
  DshotRpmCaptureDmaBuffer dshotdCaptureDmaBuffer;
#endif
};
