#include "sbusTunnelRole.hpp"
#include <algorithm>

#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"



namespace {
  SBUSConfig sbuscfg = {	//Config du sbus de la RC
    .uartd = &RoleUartDriver,
    //    .errorCb = [](SBUSError err) {DebugTrace("Err %u", err)},
    .errorCb = nullptr,
    .frameCb = &Trampoline<&SbusTunnel::maj_rc_cb_frame>::fn,
    .threadWASize = 768U,
    .externallyInverted = false,
  };
}


DeviceStatus SbusTunnel::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::SBUS_TUNNEL);
}


DeviceStatus SbusTunnel::start(UAVCAN::Node& _node)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::SBUS_TUNNEL);
  
  // use serial2 rx
  if (not boardResource.try_acquire(HR::USART_2, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  enabledChannels = decode_channel_mask(PARAM_CGET("role.tunnel.sbus.active_channels"));
  tunnelFrame.channels.len = count_active_channels(enabledChannels);
  chDbgAssert(tunnelFrame.channels.len <= SBUS_NUM_CHANNEL, "internal error");
  // if the serial telemetry is used in the future,
  // one have to add the UART and the RX pin
  //  dshotStart(&dshotd, &dshotConfig);
  //  chThdCreateStatic(waPeriodic, sizeof(waPeriodic), NORMALPRIO, &periodic, &node);
  node = &_node;
  sbusObjectInit(&sbusd);	//Init et acquisition des trames sbus
  sbusStart(&sbusd, &sbuscfg);
  sbusStartReceive(&sbusd);
  return status;
}





bool SbusTunnel::is_channel_active(ChannelMask mask, std::size_t channel_idx)
{
  return ((mask >> (channel_idx * 4)) & 0xF) != 0;
}

SbusTunnel::ChannelBitset SbusTunnel::decode_channel_mask(ChannelMask mask)
{
  ChannelBitset bits;
  for (std::size_t i = 0; i < max_channels; ++i) {
    bits[i] = is_channel_active(mask, i);
  }
  return bits;
}

uint8_t SbusTunnel::count_active_channels(const ChannelBitset& bits)
{
  return bits.count();
}

void SbusTunnel::maj_rc_cb_frame(const SBUSFrame *frame)
{
  /* tunnelFrame.channels.len */
  for (size_t i = 0; i <  tunnelFrame.channels.len; i++) {
    tunnelFrame.channels.data[i] = frame->channel[i];
  }
  tunnelFrame.flags = frame->flags;
  node->sendBroadcast(tunnelFrame, CANARD_TRANSFER_PRIORITY_LOW);
}

