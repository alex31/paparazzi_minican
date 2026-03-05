#include "roleConf.h"

#if USE_SERIAL_STREAM_ROLE

#include "serialStreamRole.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "UAVCanHelper.hpp"
#include "stdutil.h"

/**
 * @file serialStreamRole.cpp
 *
 * UART <-> uavcan.tunnel.BroadCast bridge.
 *
 * Flow:
 *  - `processUavcanToSerial` enqueues UAVCAN frames to the UART TX FIFO.
 *  - `uavcanTransmitThread` pulls UART bytes from SIO and publishes tunnel chunks.
 *  - `uartTransmitThread` dequeues tunnel frames and sends them over UART.
 *
 * Buffers:
 *  - CAN->UART: ObjectFifo<Uavcan2uart_t, 50> (DMA heap).
 *  - SIO continuous RX DMA + internal FIFO handle UART->CAN buffering.
 */


namespace {
  SIOConfig serialStreamcfg = {
    .baud = 0, // will be set at init
    .presc = USART_PRESC1,
    .cr1 = USART_CR1_RTOIE | USART_CR1_FIFOEN,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };

  constexpr SIO::DmaUserConfig serial_rx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = EXTERNAL_USART_RX_DMAMUX,
  };
  constexpr SIO::DmaUserConfig serial_tx_dma_cfg{
      .stream = STM32_DMA_STREAM_ID_ANY,
      .dmamux = EXTERNAL_USART_TX_DMAMUX,
  };

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
  DeviceStatus status(DeviceStatus::SERIAL_STREAM);
  protocol =  param_cget<"role.tunnel.serial.protocol">();
  serialStreamcfg.baud =  param_cget<"bus.serial.baudrate">();
  if (serialStreamcfg.baud == 0U) {
    DebugTrace("serial.tunnel: bus.serial.baudrate=0 is invalid for this role");
    // Let start() report a UAVCAN log when the node is fully started.
    return status;
  }

  // use serial2 TX + RX
  using HR = HWResource;
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
  
  using UavToSerialFifo = std::remove_reference_t<decltype(*fifoObjectUav2Serial)>;
  fifoObjectUav2Serial = try_new_dma<UavToSerialFifo>(DeviceStatus::SERIAL_STREAM, status);
  if (not status) {
    return status;
  }

  node.subscribeBroadcastMessages<Trampoline<&SerialStream::processUavcanToSerial>::fn>();

  if (sio_ == nullptr) {
    const SIO::ContinuousConfig cfg = {
      ExternalSIOD,
      serial_rx_dma_cfg,
      serial_tx_dma_cfg,
      serialStreamcfg,
      nullptr,
      nullptr,
      "serial rx",
      NORMALPRIO,
      THD_WORKING_AREA_SIZE(512),
    };
    sio_ = try_new_dma<SerialSIO>(DeviceStatus::SERIAL_STREAM, status, cfg);
    if (not status) {
      return status;
    }
  }
  (void)sio_->start();
  return status;
}


/**
 * @brief Starts the worker threads for UAVCAN TX and UART TX.
 */
DeviceStatus SerialStream::start(UAVCAN::Node&)
{
  if (param_cget<"bus.serial.baudrate">() == 0U) {
    DebugTrace("serial.tunnel: ROLE.tunnel.serial refused (bus.serial.baudrate=0)");
    if (m_node != nullptr) {
      UAVCAN::Helper::log(*m_node, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR,
                          "serialStreamRole.cpp::start()",
                          "bus.serial.baudrate=0 invalid for ROLE.tunnel.serial");
    }
    return DeviceStatus(DeviceStatus::SERIAL_STREAM, DeviceStatus::INVALID_PARAM);
  }
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
    SerialSIO::RxLease lease{};
    if (!sio_->receiveRx(lease, TIME_INFINITE)) {
      continue;
    }
    const uint8_t *start = lease.data;
    size_t len = lease.len;
    while (len != 0U) {
      msg.buffer.len = std::min(chunkSize, len);
      memcpy(msg.buffer.data, start, msg.buffer.len);
      while (m_node->sendBroadcast(msg) != UAVCAN::Node::CAN_OK) {
        // libcanard queue is full
        chThdSleepMilliseconds(1);
      }
      len -= msg.buffer.len;
      start += msg.buffer.len;
    }
    sio_->releaseRx(lease);
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

#endif // USE_SERIAL_STREAM_ROLE
