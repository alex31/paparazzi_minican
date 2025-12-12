#include "roleConf.h"

#if USE_RC_SBUS_ROLE

#include "sbusRole.hpp"
#include <algorithm>
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"
#include <cmath>

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif


namespace {
  SBUSConfig sbuscfg = {	//Config du sbus de la RC
    .uartd = &ExternalUARTD,
#ifdef TRACE
    .errorCb = [](SBUSError err) {DebugTrace("Sbus Err %u", err)},
#else
    .errorCb = nullptr,
#endif
    .frameCb = &Trampoline<&RC_Sbus::maj_rc_cb_frame>::fn,
    .threadWASize = 1024,
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
  
  enabledChannels = param_cget<"role.sbus.channel_mask">();
  rcInput.rcin.len = enabledChannels.count();
  rcInput.id = param_cget<"role.sbus.id">();            
  

  if (rcInput.rcin.len > SBUS_NUM_CHANNEL) {
    _node.infoCb("sbus: channel count %u exceeds max %u",
		 rcInput.rcin.len, SBUS_NUM_CHANNEL);
    return DeviceStatus(DeviceStatus::RC_SBUS, DeviceStatus::INVALID_PARAM,
			rcInput.rcin.len);
  }

  node = &_node;
  sbusObjectInit(&sbusd);	//Init et acquisition des trames sbus
  sbusStart(&sbusd, &sbuscfg);
  sbusStartReceive(&sbusd);
  return status;
}

void RC_Sbus::maj_rc_cb_frame(const SBUSFrame *frame)
{
  size_t outIdx = 0;
  const size_t maxOut = std::min<std::size_t>(rcInput.rcin.len, max_channels);
  
  for (size_t ch = 0; (ch < max_channels) && (outIdx < maxOut); ++ch) {
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

#endif // USE_RC_SBUS_ROLE
