#pragma once

#include <ch.h>
#include <hal.h>
#include <bitset>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "futabaSbusUart.h"

class SbusTunnel : public RoleBase, public RoleCrtp<SbusTunnel> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;
  void maj_rc_cb_frame(const SBUSFrame *frame);

private:
  static constexpr std::size_t max_channels = 16; // 16 voies SBUS
  using ChannelMask = std::uint64_t;
  using ChannelBitset = std::bitset<max_channels>;
  ChannelBitset enabledChannels = {};
  SBUSDriver sbusd;
  UAVCAN::Node *node;
  pprz_tunnel_SbusFrame tunnelFrame = {};

  static bool is_channel_active(ChannelMask mask, std::size_t channel_idx);
  static ChannelBitset decode_channel_mask(ChannelMask mask);
  static uint8_t count_active_channels(const ChannelBitset& bits);
};
