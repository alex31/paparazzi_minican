#include <algorithm>

#include "telemetryTunnelRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"

#include "stdutil++.hpp"



/*
  dans le subscribe : s'abonner au message paparazzi_tunnel_Telemetry
  dans la CB : creer un buffer message à partir du message UAVCAN et l'envoyer
  via uartSend -> on est synchrone, pas besoin de thread

  dans le start :
  + demarrer la liaison série avec les bon paramètres (param ??)
  + creer un thread qui :
   ° écoute sur la liaison série avec un buffer de 260 et un timout sur 1 ou 2 caractères
   ° nourrit un objet pprzLink
   ° dans la CB de pprzLink : si le message est complet/correct :
     construire un message  paparazzi_tunnel_Telemetry et l'envoyer via UAVCAN
 */

namespace {
 UARTConfig telemetrycfg =
  {
    .txend1_cb =nullptr,
    .txend2_cb = nullptr,
    .rxend_cb = nullptr,
    .rxchar_cb = nullptr,
    .rxerr_cb = nullptr,
    // .timeout_cb = [](hal_uart_driver *) {
    //   palSetLine(LINE_DBG_TIMOUT_CB);
    //   chSysPolledDelayX(1);
    //   palClearLine(LINE_DBG_TIMOUT_CB);
    // },
    .timeout_cb = nullptr,
    .timeout = 6,
    .speed = 0, // will be be at init
    .cr1 = USART_CR1_RTOIE, 
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };

}

void TelemetryTunnel::trapError_s2u(uint32_t v, uint32_t i)
{
  DebugTrace("CRC err -> %lu valid / %lu invalid", v, i); 
};



void TelemetryTunnel::processPaparazziTelemetryCommand_u2s(CanardRxTransfer *,
							   const paparazzi_tunnel_Telemetry &msg)
{
  // if (transfert->source_node_id  == m_node->getNodeId()) {
  //   DebugTrace("get my own message");
  //   return;
  // }
  
  auto copyPprzHeader = [](PprzHeader& pph, const paparazzi_tunnel_PprzHeader& src) {
    pph.source = src.source;
    pph.destination = src.destination;
    pph.classId = src.classId;
    pph.componentId = src.componentId;
    pph.msgId = src.msgId;
  };
  
  auto sendMsg = [](auto& ppmsg, size_t size) {
    uartSendTimeout(&RoleUartDriver, &size, ppmsg.data(), TIME_INFINITE);
  };

  if (msg.payload.len > XbeeMsg_t::MAX_SIZE) {
    DebugTrace("processPaparazziTelemetryCommand_u2s: (len = %u) > (MAX_SIZE = %u)",
	       msg.payload.len,  XbeeMsg_t::MAX_SIZE);
    return;
  }
						
  std::span<const uint8_t> payload(msg.payload.data, msg.payload.len);

  switch (msg.xbeeHeader.union_tag) {
  case PAPARAZZI_TUNNEL_XBEEHEADER_TX: {
    if (not xbeeFrame) {
      DebugTrace("PPRZ decoder cannoty decode XBEE frame");
      break;
    }
    XbeeHeader xbh;
    xbh.role = XbeeRole::Tx;
    xbh.frameId = msg.xbeeHeader.tx.frame_id;
    xbh.destId_bigEndian = __builtin_bswap16(msg.xbeeHeader.tx.dest_id);
    xbh.options = msg.xbeeHeader.tx.options;

    PprzHeader pph;
    copyPprzHeader(pph, msg.pprzHeader);

    auto& ppmsg = *xbeeMsg;
    PprzEncoder::genXbeeMsg(ppmsg, xbh, pph, payload);

    sendMsg(ppmsg, ppmsg.size());
    break;
  }
  case PAPARAZZI_TUNNEL_XBEEHEADER_RX: {
    if (not xbeeFrame) {
      DebugTrace("PPRZ decoder cannoty decode XBEE frame");
      break;
    }
    XbeeHeader xbh;
    xbh.role = XbeeRole::Rx;
    xbh.srcId_bigEndian = __builtin_bswap16(msg.xbeeHeader.rx.src_id);
    xbh.rssi = msg.xbeeHeader.rx.rssi;
    xbh.options = msg.xbeeHeader.rx.options;

    PprzHeader pph;
    copyPprzHeader(pph, msg.pprzHeader);

    auto& ppmsg = *xbeeMsg;
    PprzEncoder::genXbeeMsg(ppmsg, xbh, pph, payload);

    sendMsg(ppmsg, ppmsg.size());
    break;
  }
  case PAPARAZZI_TUNNEL_XBEEHEADER_NONE: {
    if (xbeeFrame) {
      DebugTrace("XBEE decoder cannoty decode PPRZ frame");
      break;
    }
    PprzHeader pph;
    copyPprzHeader(pph, msg.pprzHeader);

    auto& ppmsg = *pprzMsg;
    PprzEncoder::genPprzMsg(ppmsg, pph, payload);

    sendMsg(ppmsg, ppmsg.size());
    break;
  }
  }
}


void TelemetryTunnel::processPaparazziTelemetryCommand_s2u(PprzPolicy pol,
							   std::span<const uint8_t> in)
{
  paparazzi_tunnel_Telemetry out;

  if (pol == PprzPolicy::XBEE_API) {
    if (in.size() > XbeeMsg_t::MAX_SIZE) {
      DebugTrace("processPaparazziTelemetryCommand_s2u: (len = %u) > (MAX_SIZE = %u)",
		 in.size(),  XbeeMsg_t::MAX_SIZE);
      return;
    }
    XbeeHeader xbh(in); // will 'advance' in of sizeof XbeeHeader
    switch(xbh.role) {
    case XbeeRole::Tx: {
      out.xbeeHeader.union_tag = PAPARAZZI_TUNNEL_XBEEHEADER_TX;
      out.xbeeHeader.tx.frame_id = xbh.frameId;
      out.xbeeHeader.tx.dest_id =  __builtin_bswap16(xbh.destId_bigEndian);
      out.xbeeHeader.tx.options =  xbh.options;
      break;
    }
    case XbeeRole::Rx: {
      out.xbeeHeader.union_tag = PAPARAZZI_TUNNEL_XBEEHEADER_RX;
      out.xbeeHeader.rx.src_id = __builtin_bswap16(xbh.srcId_bigEndian);
      out.xbeeHeader.rx.rssi =  xbh.rssi;
      out.xbeeHeader.rx.options = xbh.options;
      break;
    }
    default: {
      DebugTrace("role 0x%x is neither Tx(0x1) or Rx(0x81)",
		 std::to_underlying(xbh.role));
    }
    }
  } else {
    if (in.size() >  PprzMsg_t::MAX_SIZE) {
      DebugTrace("processPaparazziTelemetryCommand_s2u: (len = %u) > (MAX_SIZE = %u)",
		 in.size(),  PprzMsg_t::MAX_SIZE);
      return;
    }

    out.xbeeHeader.union_tag = PAPARAZZI_TUNNEL_XBEEHEADER_NONE;
  }
  PprzHeader pph(in); // will 'advance' in of sizeof PprzHeader
  out.pprzHeader.source = pph.source;
  out.pprzHeader.destination = pph.destination;
  out.pprzHeader.classId = pph.classId;
  out.pprzHeader.componentId = pph.componentId;
  out.pprzHeader.msgId = pph.msgId;
  out.payload.len = in.size();
  std::copy(in.begin(), in.end(), out.payload.data);
  m_node->sendBroadcast(out, CANARD_TRANSFER_PRIORITY_MEDIUM);
}


// It is assumed that thread live forever (until poweroff) dynamic memory allocation is
// done there to not use ram if the role is not enabled. It's normal that the memory is never
// released
DeviceStatus TelemetryTunnel::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&TelemetryTunnel::processPaparazziTelemetryCommand_u2s>::fn>();

  xbeeFrame = PARAM_CGET("role.tunnel.telemetry.xbee_frame");
  if (xbeeFrame) {
    xbeeMsg = new XbeeMsg_t;
    xbS2UBuffer = new XBEEBufferType;
    
  } else {
    // pprzFrame
    pprzMsg = new PprzMsg_t;
    ppS2UBuffer = new PPRZBufferType;
  }

  telemetrycfg.speed =  PARAM_CGET("role.tunnel.telemetry.baudrate");
  uartStart(&RoleUartDriver, &telemetrycfg);
  return DeviceStatus(DeviceStatus::TELEMETRY_TUNNEL);
}


DeviceStatus TelemetryTunnel::start(UAVCAN::Node&)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::TELEMETRY_TUNNEL);
  
  
  // use serial2 tx/rx
  if (not boardResource.try_acquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "telemetry", NORMALPRIO, 
		      &Trampoline<&TelemetryTunnel::periodic>::fn, this);

  return status;
}

void TelemetryTunnel::periodic(void *)
{
  uint8_t frame[XbeeMsg_t::MAX_SIZE];
  size_t size;
  while (true) {
    size = sizeof(frame);
    uartReceiveTimeout(&RoleUartDriver, &size, frame, TIME_INFINITE);
    if (size) {
      if (xbeeFrame) {
	xbS2UBuffer->feed(std::span(frame, size));
      } else {
	ppS2UBuffer->feed(std::span(frame, size));
      }
    }
  }
}

bool TelemetryTunnel::xbeeFrame = {};
