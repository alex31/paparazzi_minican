#include "projectconf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

/*
  Todo:

  declarer 2 fifo de type input_queue_t de 2Ko chacune
  
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
  constexpr size_t recBufferSize = 320;
  constexpr size_t fifoSize = 8192;
  constexpr size_t chunkSize = sizeof(uavcan_tunnel_Broadcast{}.buffer.data);
  constexpr sysinterval_t fifoTimeout = TIME_MS2I(1);
}




void SerialStream::processUavcanToSerial(CanardRxTransfer *,
			   const uavcan_tunnel_Broadcast &msg)
{
  if (msg.protocol.protocol != protocol) {
    return;
  }

  pushToFifo(canToUartFifo, msg.buffer.data, msg.buffer.len);
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
  if (txBuffer = (uint8_t *) malloc_dma(chunkSize); !txBuffer) {
    return allocError();
  }
  if (uartToCanBuffer = (uint8_t *) malloc_m(fifoSize); !uartToCanBuffer) {
    return allocError();
  }
  if (canToUartBuffer = (uint8_t *) malloc_m(fifoSize); !canToUartBuffer) {
    return allocError();
  }

  iqObjectInit(&uartToCanFifo, uartToCanBuffer, fifoSize, nullptr, nullptr);
  iqObjectInit(&canToUartFifo, canToUartBuffer, fifoSize, nullptr, nullptr);

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

bool SerialStream::pushToFifo(input_queue_t &fifo, const uint8_t *data, size_t len)
{
  if (len == 0) {
    return true;
  }

  osalSysLock();
  if (iqGetEmptyI(&fifo) < len) {
    osalSysUnlock();
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    iqPutI(&fifo, data[i]);
  }
  osalSysUnlock();
  return true;
}

void SerialStream::uartReceiveThread(void *)
{
  while (true) {
    size_t size = recBufferSize;
    uartReceiveTimeout(&ExternalUARTD, &size, recBuffer, TIME_MS2I(1));
    pushToFifo(uartToCanFifo, recBuffer, size);
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
    msg.buffer.len = iqReadTimeout(&uartToCanFifo, msg.buffer.data, chunkSize, fifoTimeout);
    if (msg.buffer.len == 0) {
      continue;
    }
    m_node->sendBroadcast(msg);
  }
}

void SerialStream::uartTransmitThread(void *)
{
  while (true) {
    const size_t n = iqReadTimeout(&canToUartFifo, txBuffer, chunkSize, fifoTimeout);
    if (n == 0) {
      continue;
    }

    size_t toSend = n;
    uartSendTimeout(&ExternalUARTD, &toSend, txBuffer, TIME_INFINITE);
  }
}




#endif // USE_SERIAL_STREAM_ROLE
