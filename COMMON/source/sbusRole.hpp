/**
 * @file sbusRole.hpp
 * @brief SBUS receiver role definition.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include <bitset>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "futabaSbusUart.h"

/**
 * @brief Role for receiving SBUS frames and publishing RC input over UAVCAN.
 */
class RC_Sbus : public RoleBase, public RoleCrtp<RC_Sbus> {
public:
  /** @brief Subscribe to UAVCAN messages used by this role. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Initialize the SBUS driver and start processing. */
  DeviceStatus start(UAVCAN::Node& node) override;
  /** @brief Process a decoded SBUS frame. */
  void maj_rc_cb_frame(const SBUSFrame *frame);

private:
  static constexpr std::size_t max_channels = 16; // 16 voies SBUS
  using ChannelMask = std::uint64_t;
  using ChannelBitset = std::bitset<max_channels>;
  ChannelBitset enabledChannels = {};
  SBUSDriver sbusd;
  UAVCAN::Node *node;
  dronecan_sensors_rc_RCInput rcInput = {};
};
