#include "sbusRole.hpp"
#include <algorithm>
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif


namespace {
  SBUSConfig sbuscfg = {	//Config du sbus de la RC
    .uartd = &ExternalUARTD,
    //    .errorCb = [](SBUSError err) {DebugTrace("Err %u", err)},
    .errorCb = nullptr,
    .frameCb = &Trampoline<&RC_Sbus::maj_rc_cb_frame>::fn,
    .threadWASize = 768U,
    .externallyInverted = false,
  };
}


DeviceStatus RC_Sbus::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::RC_SBUS);
}


DeviceStatus RC_Sbus::start(UAVCAN::Node& _node)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::RC_SBUS);
  
  // use serial2 rx
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA10, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_1));
  }
  //  MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b01000);
#endif
  
  enabledChannels = decode_channel_mask(PARAM_CGET("role.sbus.channel_mask"));
  rcInput.rcin.len = count_active_channels(enabledChannels);
  rcInput.id = PARAM_CGET("role.sbus.id");            
  

  chDbgAssert(rcInput.rcin.len <= SBUS_NUM_CHANNEL, "internal error");
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





bool RC_Sbus::is_channel_active(ChannelMask mask, std::size_t channel_idx)
{
  return ((mask >> (channel_idx * 4)) & 0xF) != 0;
}

RC_Sbus::ChannelBitset RC_Sbus::decode_channel_mask(ChannelMask mask)
{
  ChannelBitset bits;
  for (std::size_t i = 0; i < max_channels; ++i) {
    bits[i] = is_channel_active(mask, i);
  }
  return bits;
}

uint8_t RC_Sbus::count_active_channels(const ChannelBitset& bits)
{
  return bits.count();
}

void RC_Sbus::maj_rc_cb_frame(const SBUSFrame *frame)
{
  std::size_t outIdx = 0;
  
  for (size_t ch = 0; (ch < max_channels) && (outIdx < rcInput.rcin.len); ++ch) {
    if (enabledChannels[ch]) {
      rcInput.rcin.data[outIdx++] = frame->channel[ch];
    }
  }
  rcInput.status = DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
  if (frame->flags & (1 << SBUS_FAILSAFE_BIT))
    rcInput.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;

  rcInput.quality = frame->flags & (1 << SBUS_FRAME_LOST_BIT) ? 0 : 255;
  node->sendBroadcast(rcInput, CANARD_TRANSFER_PRIORITY_HIGH);
}
