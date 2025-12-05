#include "projectconf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"
#include <new>
#include <type_traits>

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif


namespace {
  void rxchar_cb(hal_uart_driver *udp, uint16_t c);

  UARTConfig serialStreamcfg =
  {
    .txend1_cb =nullptr,
    .txend2_cb = nullptr,
    .rxend_cb = nullptr,
    .rxchar_cb = rxchar_cb,
    .rxerr_cb = nullptr,
    .timeout_cb = nullptr,
    .timeout = 6,
    .speed = 0, // will be be at init
    .cr1 = USART_CR1_RTOIE | USART_CR1_FIFOEN, 
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };

  struct UartGapCapture_t {
    input_queue_t rxq;
    uint8_t rxbuf[60];
  } *gapCapture = nullptr;

}




void SerialStream::processUavcanToSerial(CanardRxTransfer *,
			   const uavcan_tunnel_Broadcast &msg)
{
  if (msg.protocol.protocol != protocol) {
    return;
  }
  auto [status, msg_buffer] = fifoObjectUav2Serial->takeObject(TIME_IMMEDIATE);
  if (status == MSG_OK) {
    memcpy((void *) &msg_buffer, &msg, sizeof(msg));
    using MsgT = std::remove_cvref_t<decltype(msg)>;
    using BufT = std::remove_cvref_t<decltype(msg_buffer)>;
    static_assert(std::is_base_of_v<MsgT, BufT>,
		  "msg_buffer must derive from uavcan_tunnel_Broadcast");
    static_assert(std::is_trivially_copyable_v<MsgT> &&
		  std::is_trivially_copyable_v<BufT>);
    static_assert(sizeof(MsgT) <= sizeof(BufT));
    fifoObjectUav2Serial->sendObject(msg_buffer);
  } 
}


// It is assumed that thread live forever (until poweroff) dynamic memory allocation is
// done there to not use ram if the role is not enabled. It's normal that the memory is never
// released
DeviceStatus SerialStream::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  if (!fifoObjectSerial2Uav || !fifoObjectUav2Serial) {
    using SerialToUavFifo = std::remove_reference_t<decltype(*fifoObjectSerial2Uav)>;
    using UavToSerialFifo = std::remove_reference_t<decltype(*fifoObjectUav2Serial)>;
    void *serialToUavMem = malloc_dma(sizeof(SerialToUavFifo));
    void *uavToSerialMem = malloc_dma(sizeof(UavToSerialFifo));
    gapCapture = (UartGapCapture_t *) malloc_m(sizeof(UartGapCapture_t));
			  
    if (!serialToUavMem || !uavToSerialMem || !gapCapture) {
      free_dma(serialToUavMem);
      free_dma(uavToSerialMem);
      free_m(gapCapture);
      return DeviceStatus(DeviceStatus::SERIAL_STREAM, DeviceStatus::DMA_HEAP_FULL);
    }
    fifoObjectSerial2Uav = new (serialToUavMem) SerialToUavFifo();
    fifoObjectUav2Serial = new (uavToSerialMem) UavToSerialFifo();
  }
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
  iqObjectInit(&gapCapture->rxq, gapCapture->rxbuf, sizeof(gapCapture->rxbuf),
	       nullptr, nullptr);
  uartStart(&ExternalUARTD, &serialStreamcfg);
  return status;
}


DeviceStatus SerialStream::start(UAVCAN::Node&)
{
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial rx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uartReceiveThread>::fn, this);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "serial uavcan tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uavcanTransmitThread>::fn, this);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial uart tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uartTransmitThread>::fn, this);
  
  return DeviceStatus(DeviceStatus::SERIAL_STREAM);
}

void SerialStream::gatherLostBytes()
{
  etl::vector<uint8_t, sizeof(gapCapture->rxbuf)> interDmaBytes = {};

  for (int ch = iqGetTimeout(&gapCapture->rxq, TIME_IMMEDIATE);
       (ch >= 0) && !interDmaBytes.full();
       ch = iqGetTimeout(&gapCapture->rxq, TIME_IMMEDIATE)) {

    interDmaBytes.push_back((uint8_t)ch);
  }
  
  if (not interDmaBytes.empty()) {
    // best effort, if we have exhausted fifo object, nothing can be done
    // to save private ryan
    auto [status, msg_buffer] = fifoObjectSerial2Uav->takeObject(TIME_IMMEDIATE);
    if (status == MSG_OK) {
      memcpy(msg_buffer.data(), interDmaBytes.data(), interDmaBytes.size());
      msg_buffer.uninitialized_resize(interDmaBytes.size());
      fifoObjectSerial2Uav->sendObject(msg_buffer);
    }
  }
}

void SerialStream::uartReceiveThread(void *)
{
  while (true) {
    gatherLostBytes();
    auto [status, msg_buffer] = fifoObjectSerial2Uav->takeObject(TIME_INFINITE);
    if (status == MSG_OK) {
      size_t size = Uart2uavcan_t::MAX_SIZE;
      uartReceiveTimeout(&ExternalUARTD, &size, msg_buffer.data(), TIME_INFINITE);
      if (size) {
	msg_buffer.uninitialized_resize(size);
	fifoObjectSerial2Uav->sendObject(msg_buffer);
      } else {
	fifoObjectSerial2Uav->returnObject(msg_buffer);
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
    auto [status, msg_buffer] = fifoObjectSerial2Uav->receiveObject(TIME_INFINITE);
    if (status == MSG_OK) {
      uint8_t *start = msg_buffer.data();
      size_t len = msg_buffer.size();
      while (len != 0) {
	msg.buffer.len = std::min(chunkSize, len);
	memcpy(msg.buffer.data, start,  msg.buffer.len);
	while (m_node->sendBroadcast(msg) != UAVCAN::Node::CAN_OK) {
	  // libcanard queue is full
	  chThdSleepMilliseconds(1);
	}
	len -= msg.buffer.len;
	start += msg.buffer.len;
      }
      fifoObjectSerial2Uav->returnObject(msg_buffer);
    }
  }
}

void SerialStream::uartTransmitThread(void *)
{
  while (true) {
    auto [status, msg_buffer] = fifoObjectUav2Serial->receiveObject(TIME_INFINITE);
    if (status == MSG_OK) {
      size_t toSend = msg_buffer.buffer.len;
      uartSendTimeout(&ExternalUARTD, &toSend, msg_buffer.buffer.data, TIME_INFINITE);
      fifoObjectUav2Serial->returnObject(msg_buffer);
    }
  }
}

namespace {
  void rxchar_cb(hal_uart_driver *, uint16_t c) {
    chSysLockFromISR();
    iqPutI(&gapCapture->rxq, static_cast<uint8_t>(c));  // drops if full
    chSysUnlockFromISR();
  }
}

#endif // USE_SERIAL_STREAM_ROLE
