/**
 * @file serialStreamRole.hpp
 * @brief UART <-> UAVCAN tunnel bridge role definition.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "fifoObject.hpp"
#include "etl/vector.h"
#include "sioWrapper.hpp"


/**
 * @brief UART <-> uavcan.tunnel.Broadcast bridge role.
 *
 * Dynamically allocates DMA-backed FIFOs when started, and runs three threads to
 * shuttle frames between UART and UAVCAN.
 */
class SerialStream final : public RoleBase, public RoleCrtp<SerialStream> {
public:
  /// Subscribe to tunnel broadcasts and prepare UART resources.
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /// Start worker threads for UART RX, UAVCAN TX, and UART TX.
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  static constexpr size_t chunkSize = sizeof(uavcan_tunnel_Broadcast{}.buffer.data);

  /** @brief FIFO element for UART-to-UAVCAN buffering. */
  struct Uart2uavcan_t : etl::vector<uint8_t, chunkSize * 5>
  {
    Uart2uavcan_t() = default;
    Uart2uavcan_t(const Uart2uavcan_t&) = delete;
  };

  /** @brief FIFO element for UAVCAN-to-UART buffering. */
  struct Uavcan2uart_t : uavcan_tunnel_Broadcast
  {
    Uavcan2uart_t() = default;
    Uavcan2uart_t(const Uavcan2uart_t&) = delete;
    uint8_t _pad = 0;
  };

  /** @brief Handle incoming tunnel messages destined for the UART. */
  void processUavcanToSerial(CanardRxTransfer *,
			     const uavcan_tunnel_Broadcast &msg);
  /// Dequeue UART frames and publish them as tunnel chunks.
  void uavcanTransmitThread(void *);
  /// Dequeue tunnel frames and send them over UART.
  void uartTransmitThread(void *);
  /// SIO RX callback: enqueue UART bytes for CAN publishing.
  static void sioRxCb(const SIO::ByteSpan &slice, void *user);
  uint8_t protocol = 0;
  // 6Ko of dma allocated memory
  ObjectFifo<Uart2uavcan_t, 10> *fifoObjectSerial2Uav = nullptr;
  ObjectFifo<Uavcan2uart_t, 50> *fifoObjectUav2Serial = nullptr;
  static constexpr size_t dmaRxSize = chunkSize * 10U;
  static constexpr size_t dmaRxFifoDepth = 8U;
  static_assert((dmaRxSize % 2U) == 0U, "SIO DMA RX buffer must be even sized");
  using SerialSIO = SIO::Continuous<dmaRxSize, dmaRxFifoDepth>;
  SerialSIO *sio_ = nullptr;
};
