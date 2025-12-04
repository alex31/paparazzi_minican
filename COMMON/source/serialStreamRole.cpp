#include "projectconf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"
#include <type_traits>

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

/*
  Todo:

  declarer 2 fifoobject :
  dans le sens uart -> uavcan object = std::array<uint8_t, 300> et il y en a 5
  dans le sens uavcan -> uart object = std::array<uint8_t, 60> et il y en a 20
  
  dans le sens uart -> uavcan, il faut 2 threads :
  thread uartReceive
   do loop
    uartReceive[recBufferSize] -> fifo push
   end loop

  thread UAVCanTransmit
   do loop
    fifoReceive[60] avec timeout de 5ms
    sendBroadcast[effective size]
   end loop

   dans le sens uavcan -> uart, il faut 1 thread :
   dans la callback du message -> fifo push

   thread UartTransmit
   do loop
    fifoReceive[60] avec timeout de 5ms
    uartTransmit[effective size]
   end loop

  
  ---
 */


/*
  SerialStream : raw UART <-> uavcan.tunnel.Broadcast (2010) bridge

  - subscribe : listens to uavcan_tunnel_Broadcast and forwards payload to UART if protocol matches.
  - start     : configures UART (baudrate param), allocates DMA buffer, spawns thread.
  - thread    : loops on UART RX, chunks into frames, publishes uavcan_tunnel_Broadcast.
 */

namespace {
 UARTConfig serialStreamcfg =
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




void SerialStream::processUavcanToSerial(CanardRxTransfer *,
			   const uavcan_tunnel_Broadcast &msg)
{
  if (msg.protocol.protocol != protocol) {
    return;
  }
  auto [status, msg_buffer] = fifoObjectUav2Serial.takeObject(TIME_IMMEDIATE);
  if (status == MSG_OK) {
    memcpy(&msg_buffer, &msg, sizeof(msg));
    using MsgT = std::remove_cvref_t<decltype(msg)>;
    using BufT = std::remove_cvref_t<decltype(msg_buffer)>;
    static_assert(std::is_base_of_v<MsgT, BufT>,
		  "msg_buffer must derive from uavcan_tunnel_Broadcast");
    static_assert(std::is_trivially_copyable_v<MsgT> &&
		  std::is_trivially_copyable_v<BufT>);
    static_assert(sizeof(MsgT) == sizeof(BufT));
    fifoObjectUav2Serial.sendObject(msg_buffer);
  }
}


// It is assumed that thread live forever (until poweroff) dynamic memory allocation is
// done there to not use ram if the role is not enabled. It's normal that the memory is never
// released
DeviceStatus SerialStream::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  node.subscribeBroadcastMessages<Trampoline<&SerialStream::processUavcanToSerial>::fn>();
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::SERIAL_STREAM);
  
  // use serial2 TX + RX
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA09, HR::PA10, HR::F2, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_1));
  }
  //  MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b01100);
#endif
  
  protocol =  PARAM_CGET("role.tunnel.serial.protocol");
  serialStreamcfg.speed =  PARAM_CGET("bus.serial.baudrate");
  uartStart(&ExternalUARTD, &serialStreamcfg);
  return status;
}


DeviceStatus SerialStream::start(UAVCAN::Node&)
{
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial rx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uartReceiveThread>::fn, this);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial uavcan tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uavcanTransmitThread>::fn, this);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial uart tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uartTransmitThread>::fn, this);
  
  return DeviceStatus(DeviceStatus::SERIAL_STREAM);
}


void SerialStream::uartReceiveThread(void *)
{
  while (true) {
    auto [status, msg_buffer] = fifoObjectSerial2Uav.takeObject(TIME_INFINITE);
    if (status == MSG_OK) {
      size_t size = Uart2uavcan_t::MAX_SIZE;
      uartReceiveTimeout(&ExternalUARTD, &size, msg_buffer.data(), TIME_INFINITE);
      if (size) {
	msg_buffer.uninitialized_resize(size);
	fifoObjectSerial2Uav.sendObject(msg_buffer);
      } else {
	fifoObjectSerial2Uav.returnObject(msg_buffer);
      }
    }
  }
}

void SerialStream::uavcanTransmitThread(void *)
{
  uavcan_tunnel_Broadcast msg = {
    .protocol = {.protocol = protocol},
    .channel_id = PLATFORM_MINICAN ? 2 : 1,
    .buffer = {}
  };

  while (true) {
    auto [status, msg_buffer] = fifoObjectSerial2Uav.receiveObject(TIME_INFINITE);
    if (status == MSG_OK) {
      uint8_t *start = msg_buffer.data();
      size_t len = msg_buffer.size();
      while (len != 0) {
	msg.buffer.len = std::min(chunkSize, len);
	memcpy(msg.buffer.data, start,  msg.buffer.len);
	m_node->sendBroadcast(msg);
	len -= msg.buffer.len;
	start += msg.buffer.len;
      }
      fifoObjectSerial2Uav.returnObject(msg_buffer);
    }
  }
}

void SerialStream::uartTransmitThread(void *)
{
  while (true) {
    auto [status, msg_buffer] = fifoObjectUav2Serial.receiveObject(TIME_INFINITE);
    if (status == MSG_OK) {
      size_t toSend = msg_buffer.buffer.len;
      uartSendTimeout(&ExternalUARTD, &toSend, msg_buffer.buffer.data, TIME_INFINITE);
      fifoObjectUav2Serial.returnObject(msg_buffer);
    }
  }
}

ObjectFifo<SerialStream::Uart2uavcan_t, 5>  IN_DMA_SECTION(SerialStream::fifoObjectSerial2Uav);
ObjectFifo<SerialStream::Uavcan2uart_t, 25> IN_DMA_SECTION(SerialStream::fifoObjectUav2Serial);


#endif // USE_SERIAL_STREAM_ROLE
