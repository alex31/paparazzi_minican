#include "escDshotRole.hpp"
#include <algorithm>
#include "ressourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil++.hpp"




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
  mapIndex1 = PARAM_CGET("role.esc.dshot.map_index1");
  numChannels = PARAM_CGET("role.esc.dshot.num_channels");
  loopPeriod = PARAM_CGET("role.esc.dshot.cmd_rate");
  rpmFrqDiv = PARAM_CGET("role.esc.dshot.rpm_freq_div");

  chDbgAssert(numChannels <= 4, "too many channels for dshot esc");

  dshotConfig = {
    .dma_stream = STM32_DMA_STREAM_ID_ANY,
    .dmamux =  DSHOT_EMIT_STREAM(SRV1_TIM),
    .pwmp = &PWMD1,
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
  if (not boardResource.try_acquire(HR::TIM_1, HR::TIM_7)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }

  // only the pins that are actually in use depending on role.pwm.num_servos
  for (uint8_t channel=0; channel < numChannels; channel++) {
    if (not boardResource.try_acquire(static_cast<HR>(std::to_underlying(HR::PA08) + channel))) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
    }
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
    for (size_t idx = 0; idx < numChannels; ++idx) {
      throttles[idx] = std::clamp((msg.cmd.data[idx + mapIndex1]) / 4, 0,
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
    
  auto now = chVTGetSystemTimeX();
  while(chTimeDiffX(chVTGetSystemTimeX(), now) < TIME_MS2I(1000)) {
    dshotSendSpecialCommand(&dshotd, DSHOT_ALL_MOTORS, DSHOT_CMD_MOTOR_STOP);
    chThdSleepMicroseconds(500);
#if DSHOT_BIDIR
    dshotSendSpecialCommand(&dshotd, DSHOT_ALL_MOTORS, DSHOT_CMD_BIDIR_EDT_MODE_ON);
    chThdSleepMicroseconds(500);
#endif
  }
    
  while(true) {
    const systime_t ts = chVTGetSystemTimeX();
    for (size_t i = 0; i <  numChannels; i++) {
      dshotSetThrottle(&dshotd, i, throttles[i]);
    }
    dshotSendFrame(&dshotd);
#if DSHOT_BIDIR
    if ((rpmFrqDiv != 0) && ((++count % rpmFrqDiv) == 0)) {
      for (size_t i = 0; i <  numChannels; i++) {
	if  (const uint32_t readErpm = dshotGetRpm(&dshotd, i);
	     (readErpm != DSHOT_BIDIR_ERR_CRC) && (readErpm != DSHOT_BIDIR_TLM_EDT)) {
	  msgEscStatus.esc_index = i + mapIndex1;
	  msgEscStatus.rpm = readErpm;
#if	DSHOT_BIDIR_EXTENTED_TELEMETRY
	  const DshotTelemetry tlm = dshotGetTelemetry(&dshotd, i);
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

