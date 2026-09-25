/**
 * @file escDshotRole.cpp
 * @brief DShot ESC role implementation.
 */
#include "roleConf.h"

#if USE_ESC_DSHOT_ROLE

#include "escDshotRole.hpp"
#include "resourceManager.hpp"
#include "hardwareConf.hpp"
#include <limits>

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)

namespace {
  /// PWM driver used for DShot output.
  static constexpr PWMDriver &DSHOT_PWMD = CONCAT(PWMD, SRV1_TIM);
}



/** @brief Register UAVCAN subscriptions for ESC command handling. */
DeviceStatus EscDshot::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&EscDshot::processEscRawCommand>::fn>();
  return DeviceStatus::ESC_DSHOT;
}

/** @brief Configure DShot hardware resources and start the worker thread. */
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
  const uint16_t motorPoles = param_cget<"role.esc.dshot.motor_poles">();
  if ((motorPoles < 2) || ((motorPoles % 2) != 0)) {
    if (m_node) {
      m_node->infoCb("esc.dshot: motor_poles must be even and >= 2 (got %u)", motorPoles);
    }
    return DeviceStatus(DeviceStatus::ESC_DSHOT, DeviceStatus::INVALID_PARAM, motorPoles);
  }
  polePairs = motorPoles / 2;

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

  // only acquire pins that are in use depending on role.esc.dshot.channel_mask
  for (uint8_t channelIdx = 0; channelIdx < numChannels; ++channelIdx) {
    const uint8_t channel = channelMap[channelIdx];
    const auto pinRes = static_cast<HR>(std::to_underlying(HR::PA08) + channel);
    if (not boardResource.tryAcquire(pinRes)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
                          std::to_underlying(pinRes));
    }
  }

  // if the serial telemetry is used in the future,
  // one have to add the UART and the RX pin
  dshotStart(&dshotd, &dshotConfig);
#if DSHOT_BIDIR && DSHOT_BIDIR_EXTENTED_TELEMETRY
  // FlexDebug's generic encoder reserves its maximum 258-byte buffer on the stack.
  constexpr size_t stackSize = 1024;
#else
  constexpr size_t stackSize = 512;
#endif
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(stackSize), "periodic dshot", NORMALPRIO,
		      &Trampoline<&EscDshot::periodic>::fn, this);


  return status;
}
/** @brief Decode and normalize raw ESC commands into DShot throttle values. */
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

/** @brief DShot output loop with optional bidirectional telemetry. */
void  EscDshot::periodic(void *)	
{
#if  DSHOT_BIDIR
  uint32_t count = 0;
  uavcan_equipment_esc_Status msgEscStatus = {};
  std::array<uint32_t, DSHOT_CHANNELS> pendingErpm;
  pendingErpm.fill(DSHOT_BIDIR_ERR_CRC);
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
  struct PendingEdt {
    uint8_t mask = 0, stress = 0, status = 0;
  } pendingEdt[DSHOT_CHANNELS];
  systime_t lastDebugPublish = chVTGetSystemTimeX();
#endif
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

    const bool publishTelemetry = (rpmFrqDiv != 0) && ((++count % rpmFrqDiv) == 0);
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    const bool publishDebug = publishTelemetry &&
      (chTimeDiffX(lastDebugPublish, ts) >= TIME_MS2I(100));
#endif
    for (size_t slot = 0; slot < numChannels; ++slot) {
      const uint8_t channel = channelMap[slot];
      // Every response must be processed: dshotGetRpm also updates EDT data.
      const uint32_t readErpm = dshotGetRpm(&dshotd, channel);
      if ((readErpm != DSHOT_BIDIR_ERR_CRC) && (readErpm != DSHOT_BIDIR_TLM_EDT)) {
        pendingErpm[channel] = readErpm;
      }
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
      DshotTelemetry tlm = {};
      auto &pending = pendingEdt[channel];
      if ((readErpm == DSHOT_BIDIR_TLM_EDT) || publishTelemetry) {
        tlm = dshotGetTelemetry(&dshotd, channel);
        pending.mask &= tlm.valid_mask;
        if (readErpm == DSHOT_BIDIR_TLM_EDT) {
          if (tlm.updated_mask & (1U << DSHOT_TELEM_STRESS)) {
            pending.stress = (pending.mask & (1U << DSHOT_TELEM_STRESS)) ?
              std::max(pending.stress, tlm.stress) : tlm.stress;
            pending.mask |= 1U << DSHOT_TELEM_STRESS;
          }
          if (tlm.updated_mask & (1U << DSHOT_TELEM_STATUS)) {
            if (!(pending.mask & (1U << DSHOT_TELEM_STATUS))) pending.status = 0;
            // Latch events and the four-bit maximum stress; bit 4 is reserved.
            pending.status = ((pending.status | tlm.status) & 0xE0U) |
              std::max(pending.status & 0x0FU, tlm.status & 0x0FU);
            pending.mask |= 1U << DSHOT_TELEM_STATUS;
          }
        }
      }
#endif
      if (publishTelemetry && (pendingErpm[channel] != DSHOT_BIDIR_ERR_CRC)) {
        msgEscStatus.esc_index = slot + mapIndex1;
        msgEscStatus.rpm = pendingErpm[channel] / polePairs;
        msgEscStatus.voltage = msgEscStatus.current = msgEscStatus.temperature =
          std::numeric_limits<float>::quiet_NaN();
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
        if (dshotTelemetryIsValid(&tlm, DSHOT_TELEM_VOLTAGE))
          msgEscStatus.voltage = tlm.frame.voltage / 100.0f;
        if (dshotTelemetryIsValid(&tlm, DSHOT_TELEM_CURRENT))
          msgEscStatus.current = tlm.frame.current / 100.0f;
        if (dshotTelemetryIsValid(&tlm, DSHOT_TELEM_TEMP))
          msgEscStatus.temperature = tlm.frame.temp + 273.15f;
#endif  // TELEMETRY
        m_node->sendBroadcast(msgEscStatus, CANARD_TRANSFER_PRIORITY_LOW);
      }
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
      if (publishDebug && pending.mask != 0) {
        // Private MicroCAN encoding v1; see docs/software/roles/dshot_telemetry.md.
        // A separate ID per ESC prevents receiver caches from merging channels.
        msgEscDebug.id = 2000U + slot + mapIndex1;
        msgEscDebug.u8.len = 5;
        auto *data = msgEscDebug.u8.data;
        const bool hasStress = pending.mask & (1U << DSHOT_TELEM_STRESS);
        const bool hasStatus = pending.mask & (1U << DSHOT_TELEM_STATUS);
        data[0] = 1;
        data[1] = slot + mapIndex1;
        data[2] = hasStress | (hasStatus << 1);
        data[3] = hasStress ? pending.stress : 0;
        data[4] = hasStatus ? pending.status : 0;
        if (m_node->sendBroadcast(msgEscDebug, CANARD_TRANSFER_PRIORITY_LOW) == UAVCAN::Node::CAN_OK) {
          pending = {}; // Retain events if the CAN queue rejects the transfer.
        }
      }
#endif
    }
    // Do not republish an old RPM if no valid RPM arrives in the next interval.
    if (publishTelemetry) {
      pendingErpm.fill(DSHOT_BIDIR_ERR_CRC);
    }
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    if (publishDebug) lastDebugPublish = ts;
#endif
#endif // BIDIR
    chThdSleepUntilWindowed(ts, ts + loopPeriod);
  }
}

#endif // USE_ESC_DSHOT_ROLE
