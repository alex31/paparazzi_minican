#include "roleConf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "stdutil++.hpp"
#include "hal_stm32_dma.h"
#include <algorithm>
#include <cstring>
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
 *  - `sioRxCb` is called from the SIO RX worker thread and enqueues UART bytes for CAN.
 *  - `uavcanTransmitThread` splits UART frames into tunnel chunks and publishes them.
 *  - `uartTransmitThread` dequeues tunnel frames and sends them over UART.
 *
 * Buffers:
 *  - UART->CAN: ObjectFifo<Uart2uavcan_t, 10> (DMA heap).
 *  - CAN->UART: ObjectFifo<Uavcan2uart_t, 50> (DMA heap).
 *  - SIO continuous RX DMA handles idle flush and buffering.
 */


namespace {
  SIOConfig serialStreamcfg = {
    .baud = 0, // will be set at init
    .presc = USART_PRESC1,
    .cr1 = USART_CR1_RTOIE | USART_CR1_FIFOEN,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };

#if PLATFORM_MINICAN
  constexpr SIO::DmaUserConfig serial_rx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = STM32_DMAMUX1_USART2_RX,
  };
  constexpr SIO::DmaUserConfig serial_tx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = STM32_DMAMUX1_USART2_TX,
  };
#endif

#if PLATFORM_MICROCAN
  constexpr SIO::DmaUserConfig serial_rx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = STM32_DMAMUX1_USART1_RX,
  };
  constexpr SIO::DmaUserConfig serial_tx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = STM32_DMAMUX1_USART1_TX,
  };
#endif

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
  auto msg_opt = fifoObjectUav2Serial->takeObject(TIME_IMMEDIATE);
  if (msg_opt) {
    auto &msg_buffer = msg_opt->get();
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

  fifoObjectSerial2Uav = new (serialToUavMem) SerialToUavFifo();
  fifoObjectUav2Serial = new (uavToSerialMem) UavToSerialFifo();

  node.subscribeBroadcastMessages<Trampoline<&SerialStream::processUavcanToSerial>::fn>();
  DeviceStatus status(DeviceStatus::SERIAL_STREAM);
  
 
  protocol =  param_cget<"role.tunnel.serial.protocol">();
  serialStreamcfg.baud =  param_cget<"bus.serial.baudrate">();

  if (sio_ == nullptr) {
    void *sio_mem = malloc_m(sizeof(SerialSIO));
    if (!sio_mem) {
      return DeviceStatus(DeviceStatus::SERIAL_STREAM, DeviceStatus::HEAP_FULL);
    }
    const SIO::ContinuousConfig cfg = {
      ExternalSIOD,
      serial_rx_dma_cfg,
      serial_tx_dma_cfg,
      serialStreamcfg,
      &SerialStream::sioRxCb,
      this,
      "serial rx",
      NORMALPRIO,
      THD_WORKING_AREA_SIZE(512),
    };
    sio_ = new (sio_mem) SerialSIO(cfg);
  }
  (void)sio_->start();
  return status;
}


/**
 * @brief Starts the worker threads for UART RX, UAVCAN TX, and UART TX.
 */
DeviceStatus SerialStream::start(UAVCAN::Node&)
{
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "serial uavcan tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uavcanTransmitThread>::fn, this);
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "serial uart tx", NORMALPRIO, 
		      &Trampoline<&SerialStream::uartTransmitThread>::fn, this);
  
  return DeviceStatus(DeviceStatus::SERIAL_STREAM);
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
    .channel_id = static_cast<uint8_t>(param_cget<"role.tunnel.serial.channel_id">()),
    .buffer = {}
  };

  while (true) {
    auto msg_opt = fifoObjectSerial2Uav->receiveObject(TIME_INFINITE);
    if (msg_opt) {
      auto &msg_buffer = msg_opt->get();
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
    auto msg_opt = fifoObjectUav2Serial->receiveObject(TIME_INFINITE);
    if (msg_opt) {
      auto &msg_buffer = msg_opt->get();
      const SIO::ByteSpan slice(msg_buffer.buffer.data, msg_buffer.buffer.len);
      (void)sio_->writeTimeout(slice, TIME_INFINITE);
      fifoObjectUav2Serial->returnObject(msg_buffer);
    }
  }
}

namespace {
}

void SerialStream::sioRxCb(const SIO::ByteSpan &slice, void *user)
{
  auto *self = static_cast<SerialStream *>(user);
  if ((self == nullptr) || (self->fifoObjectSerial2Uav == nullptr)) {
    return;
  }

  size_t offset = 0U;
  while (offset < slice.size()) {
    auto msg_opt = self->fifoObjectSerial2Uav->takeObject(TIME_IMMEDIATE);
    if (!msg_opt) {
      break;
    }
    auto &msg_buffer = msg_opt->get();
    const size_t to_copy = std::min(Uart2uavcan_t::MAX_SIZE, slice.size() - offset);
    std::memcpy(msg_buffer.data(), slice.data() + offset, to_copy);
    msg_buffer.uninitialized_resize(to_copy);
    self->fifoObjectSerial2Uav->sendObject(msg_buffer);
    offset += to_copy;
  }
}

#endif // USE_SERIAL_STREAM_ROLE
