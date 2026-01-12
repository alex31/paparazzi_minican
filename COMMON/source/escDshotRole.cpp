#include "roleConf.h"

#if USE_ESC_DSHOT_ROLE

#include "escDshotRole.hpp"
#include <algorithm>
#include "resourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil++.hpp"
#include "hardwareConf.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)

namespace {
  static constexpr PWMDriver &DSHOT_PWMD = CONCAT(PWMD, SRV1_TIM);
}



DeviceStatus EscDshot::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&EscDshot::processEscRawCommand>::fn>();
  return DeviceStatus::ESC_DSHOT;
}

DeviceStatus EscDshot::start(UAVCAN::Node& /*node*/)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::ESC_DSHOT);
  mapIndex1 = param_cget<"role.esc.dshot.map_index1">();
  const uint8_t configuredMask =
    static_cast<uint8_t>(param_cget<"role.esc.dshot.channel_mask">()) & 0b1111;
  channelMap = configuredMask;
  numChannels = channelMap.count;
  loopPeriod = param_cget<"role.esc.dshot.cmd_rate">();
  rpmFrqDiv = param_cget<"role.esc.dshot.rpm_freq_div">();

  if ((numChannels == 0) || (numChannels > 4)) {
    if (m_node) {
      m_node->infoCb("esc.dshot: invalid channel mask=0x%02x", channelMap.mask);
    }
    return DeviceStatus(DeviceStatus::ESC_DSHOT, DeviceStatus::INVALID_PARAM, channelMap.mask);
  }

  dshotConfig = {
    .dma_stream = STM32_DMA_STREAM_ID_ANY,
    .dmamux =  DSHOT_EMIT_STREAM(SRV1_TIM),
    .pwmp = &DSHOT_PWMD,
    .tlm_sd = NULL,
    .dma_command = &dshotdDmaBuffer,
#if DSHOT_BIDIR
    .dma_capt_cfg =  {
      .gptd = &GPTD7,
      .dma_streams = {DSHOTS_CAPTURE_STREAMS(SRV1_TIM)},
      .dma_capture = &dshotdCaptureDmaBuffer
    },
#endif
  };


  // use timer
  if (not boardResource.tryAcquire(HR::TIM_1, HR::TIM_7)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                        std::to_underlying(HR::TIM_1));
  }

#if PLATFORM_MICROCAN
  DynPin::setScenario(DynPin::Scenario::PWM, channelMap.mask);
#endif

  // only acquire pins that are in use depending on role.esc.dshot.channel_mask
  for (uint8_t channelIdx = 0; channelIdx < numChannels; ++channelIdx) {
    const uint8_t channel = channelMap[channelIdx];
    const auto pinRes = static_cast<HR>(std::to_underlying(HR::PA08) + channel);
    if (not boardResource.tryAcquire(pinRes)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                          std::to_underlying(pinRes));
    }
#if PLATFORM_MICROCAN
    const auto altRes = static_cast<HR>(std::to_underlying(HR::F1) + channel);
    if (not boardResource.tryAcquire(altRes)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                          std::to_underlying(altRes));
    }
#endif
  }

  // if the serial telemetry is used in the future,
  // one have to add the UART and the RX pin
  dshotStart(&dshotd, &dshotConfig);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "periodic dshot", NORMALPRIO, 
		      &Trampoline<&EscDshot::periodic>::fn, this);


  return status;
}





void EscDshot::processEscRawCommand(CanardRxTransfer* /*transfer*/,
					  const  uavcan_equipment_esc_RawCommand &msg)
{
  // raw command are in the range [-8192, 8191]
  // we normalize and clamp to the range [ 0; 2000]
  if ((mapIndex1 + numChannels) <= msg.cmd.len) {
    for (size_t slot = 0; slot < numChannels; ++slot) {
      throttles[slot] = std::clamp((msg.cmd.data[slot + mapIndex1]) / 4, 0,
					     2000);
	  }
	}
}

void  EscDshot::periodic(void *)	
{
#if  DSHOT_BIDIR
  uint32_t count = 0;
  uavcan_equipment_esc_Status msgEscStatus = {};
#endif
    
  const auto start = chVTGetSystemTimeX();
  while(chTimeDiffX(start, chVTGetSystemTimeX()) < TIME_MS2I(1000)) {
    for (size_t slot = 0; slot < numChannels; ++slot) {
      dshotSendSpecialCommand(&dshotd, channelMap[slot], DSHOT_CMD_MOTOR_STOP);
    }
    chThdSleepMicroseconds(500);
#if DSHOT_BIDIR
    for (size_t slot = 0; slot < numChannels; ++slot) {
      dshotSendSpecialCommand(&dshotd, channelMap[slot], DSHOT_CMD_BIDIR_EDT_MODE_ON);
    }
    chThdSleepMicroseconds(500);
#endif
  }
  
  while(true) {
    const systime_t ts = chVTGetSystemTimeX();
    uint16_t perChannelThrottle[4] = {0, 0, 0, 0};
    for (size_t slot = 0; slot < numChannels; ++slot) {
      perChannelThrottle[channelMap[slot]] = throttles[slot];
    }
    for (uint8_t channel = 0; channel < 4; ++channel) {
      dshotSetThrottle(&dshotd, channel, perChannelThrottle[channel]);
    }
    dshotSendFrame(&dshotd);
#if DSHOT_BIDIR
    if ((rpmFrqDiv != 0) && ((++count % rpmFrqDiv) == 0)) {
      for (size_t slot = 0; slot <  numChannels; ++slot) {
        const uint8_t channel = channelMap[slot];
	if  (const uint32_t readErpm = dshotGetRpm(&dshotd, channel);
	     (readErpm != DSHOT_BIDIR_ERR_CRC) && (readErpm != DSHOT_BIDIR_TLM_EDT)) {
	  msgEscStatus.esc_index = slot + mapIndex1;
	  msgEscStatus.rpm = readErpm;
#if	DSHOT_BIDIR_EXTENTED_TELEMETRY
	  const DshotTelemetry tlm = dshotGetTelemetry(&dshotd, channel);
	  msgEscStatus.voltage =  tlm.frame.voltage / 100.0f;
	  msgEscStatus.current =  tlm.frame.current / 100.0f;
	  msgEscStatus.temperature = tlm.frame.temp;
#endif  // TELEMETRY
	  m_node->sendBroadcast(msgEscStatus, CANARD_TRANSFER_PRIORITY_LOW);
	}
      }
    }
#endif // BIDIR
    chThdSleepUntilWindowed(ts, ts + loopPeriod);
  }
}

#endif // USE_ESC_DSHOT_ROLE
