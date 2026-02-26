/**
 * @file serialStreamRole.hpp
 * @brief UART <-> UAVCAN tunnel bridge role definition.
 */
#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "sioWrapper.hpp"


/**
 * @brief UART <-> uavcan.tunnel.Broadcast bridge role.
 *
 * Dynamically allocates DMA-backed FIFOs when started, and runs worker threads to
 * shuttle frames between UART and UAVCAN.
 */
class SerialStream final : public RoleBase, public RoleCrtp<SerialStream> {
public:
  /// Subscribe to tunnel broadcasts and prepare UART resources.
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /// Start worker threads for UAVCAN TX and UART TX.
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  static constexpr size_t chunkSize = sizeof(uavcan_tunnel_Broadcast{}.buffer.data);

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
  uint8_t protocol = 0;
  // DMA-heap allocated queue for UAVCAN->UART path.
  ObjectFifo<Uavcan2uart_t, 50> *fifoObjectUav2Serial = nullptr;
  // Keep about 2 KiB of queued UART->CAN payload in SIO internal FIFO.
  static constexpr size_t dmaRxSize = 512U;
  static constexpr size_t dmaRxFifoDepth = 8U;
  static_assert((dmaRxSize % 2U) == 0U, "SIO DMA RX buffer must be even sized");
  using SerialSIO = SIO::Continuous<dmaRxSize, dmaRxFifoDepth>;
  SerialSIO *sio_ = nullptr;
};
