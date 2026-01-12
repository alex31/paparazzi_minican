#include "roleConf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "stdutil++.hpp"
#include <new>
#include <type_traits>

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

/**
 * @file serialStreamRole.cpp
 *
 * UART <-> uavcan.tunnel.BroadCast bridge.
 *
 * Flow:
 *  - `processUavcanToSerial` enqueues UAVCAN frames to the UART TX FIFO.
 *  - `uartReceiveThread` continuously DMA-reads from UART and enqueues frames to CAN.
 *  - `gatherLostBytes` drains bytes received between DMA armings (via `rxchar_cb`)
 *    and packs them into the FIFO when a slot is available.
 *  - `uavcanTransmitThread` splits UART frames into tunnel chunks and publishes them.
 *  - `uartTransmitThread` dequeues tunnel frames and sends them over UART.
 *
 * Buffers:
 *  - UART->CAN: ObjectFifo<Uart2uavcan_t, 10> (DMA heap) + small gap input_queue.
 *  - CAN->UART: ObjectFifo<Uavcan2uart_t, 50> (DMA heap).
 *  - Hardware UART FIFO is enabled with 1-byte thresholds to preserve RXNE semantics.
 */


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

  /// Small queue to capture bytes that arrive between DMA armings.
  struct UartGapCapture_t {
    input_queue_t rxq;
    uint8_t rxbuf[60];
  } *gapCapture = nullptr;

}

/**
 * @brief UAVCAN subscriber callback: queue tunnel frames for UART TX.
 *
 * Drops frames whose tunnel protocol does not match. Copies the message into a
 * pooled UART TX object and enqueues it if a slot is available.
 */
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


/**
 * @brief UAVCAN subscription handler. Bridges tunnel frames to the UART TX FIFO.
 *
 * @details Drops frames with protocol mismatch, copies the payload into a
 *          pooled UART TX object, then queues it for uartTransmitThread.
 */
DeviceStatus SerialStream::subscribe(UAVCAN::Node& node)
{
  m_node = &node;
  // use serial2 TX + RX
  using HR = HWResource;
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
  
  using SerialToUavFifo = std::remove_reference_t<decltype(*fifoObjectSerial2Uav)>;
  using UavToSerialFifo = std::remove_reference_t<decltype(*fifoObjectUav2Serial)>;
  void *serialToUavMem = malloc_dma(sizeof(SerialToUavFifo));
  void *uavToSerialMem = malloc_dma(sizeof(UavToSerialFifo));
  if (!serialToUavMem || !uavToSerialMem) {
    free_dma(serialToUavMem);
    free_dma(uavToSerialMem);
    return DeviceStatus(DeviceStatus::SERIAL_STREAM, DeviceStatus::DMA_HEAP_FULL);
  }

  gapCapture = (UartGapCapture_t *) malloc_m(sizeof(UartGapCapture_t));
  if (!gapCapture) {
    free_dma(serialToUavMem);
    free_dma(uavToSerialMem);
    free_m(gapCapture);
    return DeviceStatus(DeviceStatus::SERIAL_STREAM, DeviceStatus::HEAP_FULL);
  }
  
  fifoObjectSerial2Uav = new (serialToUavMem) SerialToUavFifo();
  fifoObjectUav2Serial = new (uavToSerialMem) UavToSerialFifo();

  node.subscribeBroadcastMessages<Trampoline<&SerialStream::processUavcanToSerial>::fn>();
  DeviceStatus status(DeviceStatus::SERIAL_STREAM);
  
 
  protocol =  param_cget<"role.tunnel.serial.protocol">();
  serialStreamcfg.speed =  param_cget<"bus.serial.baudrate">();
  iqObjectInit(&gapCapture->rxq, gapCapture->rxbuf, sizeof(gapCapture->rxbuf),
	       nullptr, nullptr);
  uartStart(&ExternalUARTD, &serialStreamcfg);
  return status;
}


/**
 * @brief Starts the worker threads for UART RX, UAVCAN TX, and UART TX.
 */
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

/**
 * @brief Pack bytes captured between DMA armings into a FIFO object.
 *
 * Best effort: if no free FIFO object is available, the captured bytes are dropped.
 */
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

/**
 * @brief UART RX loop: drain gap bytes, arm DMA receive, enqueue completed frames.
 */
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

/**
 * @brief Dequeue UART frames and send as uavcan.tunnel.Broadcast chunks.
 *
 * Retries if the CAN TX queue is temporarily full.
 */
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

/**
 * @brief Dequeue CAN-originated frames and send them over UART.
 */
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
