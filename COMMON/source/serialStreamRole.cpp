#include "projectconf.h"

#if USE_SERIAL_STREAM_ROLE

#include <algorithm>
#include <new>
#include <cstring>

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif



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
  constexpr size_t recBufferSize = 300;
}




void SerialStream::processUavcanToSerial(CanardRxTransfer *,
			   const uavcan_tunnel_Broadcast &msg)
{
  if (msg.protocol.protocol == protocol) {
    size_t len = msg.buffer.len;
    uartSendTimeout(&ExternalUARTD, &len, msg.buffer.data, TIME_INFINITE);
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
  
  auto allocError = [] {
    return DeviceStatus(DeviceStatus::MEMORY, DeviceStatus::HEAP_FULL);
  };

  if (recBuffer = (uint8_t *) malloc_dma(recBufferSize); !recBuffer) {
    return allocError();
  }

  protocol =  PARAM_CGET("role.tunnel.serial.protocol");
  serialStreamcfg.speed =  PARAM_CGET("bus.serial.baudrate");
  uartStart(&ExternalUARTD, &serialStreamcfg);
  return status;
}


DeviceStatus SerialStream::start(UAVCAN::Node&)
{
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "serial stream", NORMALPRIO, 
		      &Trampoline<&SerialStream::periodic>::fn, this);
  
  return DeviceStatus(DeviceStatus::SERIAL_STREAM);
}

void SerialStream::periodic(void *)
{
  size_t size;
  uavcan_tunnel_Broadcast msg = {
    .protocol = {.protocol = protocol},
    .channel_id = PLATFORM_MINICAN ? 2 : 1,
    .buffer = {}
  };
  
  while (true) {
    size = recBufferSize;
    uartReceiveTimeout(&ExternalUARTD, &size, recBuffer, TIME_INFINITE);
    uint8_t *begin = recBuffer;
    while (size) {
      msg.buffer.len = std::min(size, std::size(uavcan_tunnel_Broadcast{}.buffer.data));
      memcpy(msg.buffer.data, begin, msg.buffer.len);
      begin += msg.buffer.len;
      size -= msg.buffer.len;
      m_node->sendBroadcast(msg);
    }
  }
}




#endif // USE_SERIAL_STREAM_ROLE
