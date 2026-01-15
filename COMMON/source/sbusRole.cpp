#include "roleConf.h"

#if USE_RC_SBUS_ROLE

#include "sbusRole.hpp"
#include <algorithm>
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
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

  static constexpr uint32_t SBUS_RANGE_MIN   = 200;
  static constexpr uint32_t SBUS_RANGE_MAX   = 1800;
  static constexpr uint32_t SBUS_TARGET_MIN  = 1000;
  static constexpr uint32_t SBUS_TARGET_MAX  = 2000;


  
  uint32_t scale_to_pwm_microseconds(uint32_t sbus);
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
      rcInput.rcin.data[outIdx++] = scale_to_pwm_microseconds(frame->channel[ch]);
    }
  }
  rcInput.status = DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
  if (frame->flags & (1 << SBUS_FAILSAFE_BIT))
    rcInput.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;
  
  rcInput.quality = frame->flags & (1 << SBUS_FRAME_LOST_BIT) ? 0 : 255;
  node->sendBroadcast(rcInput, CANARD_TRANSFER_PRIORITY_HIGH);
}


namespace {
  static constexpr uint32_t SBUS_RANGE_RANGE  = SBUS_RANGE_MAX - SBUS_RANGE_MIN;
  static constexpr uint32_t SBUS_TARGET_RANGE = SBUS_TARGET_MAX - SBUS_TARGET_MIN;

  uint32_t scale_to_pwm_microseconds(uint32_t sbus_raw) {
    const uint32_t r = std::clamp(sbus_raw, SBUS_RANGE_MIN, SBUS_RANGE_MAX);
    const uint32_t num = (r - SBUS_RANGE_MIN) * SBUS_TARGET_RANGE;
    return num / SBUS_RANGE_RANGE + SBUS_TARGET_MIN;
  }

}

#endif // USE_RC_SBUS_ROLE
