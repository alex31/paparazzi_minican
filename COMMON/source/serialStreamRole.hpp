#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "fifoObject.hpp"
#include "etl/vector.h"


class SerialStream final : public RoleBase, public RoleCrtp<SerialStream> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  static constexpr size_t chunkSize = sizeof(uavcan_tunnel_Broadcast{}.buffer.data);

  struct Uart2uavcan_t : etl::vector<uint8_t, chunkSize * 5>
  {
    Uart2uavcan_t() = default;
    Uart2uavcan_t(const Uart2uavcan_t&) = delete;
  };

  struct Uavcan2uart_t : uavcan_tunnel_Broadcast
  {
    Uavcan2uart_t() = default;
    Uavcan2uart_t(const Uavcan2uart_t&) = delete;
    uint8_t _pad = 0;
  };


  void processUavcanToSerial(CanardRxTransfer *,
			     const uavcan_tunnel_Broadcast &msg);
  void uartReceiveThread(void *);
  void uavcanTransmitThread(void *);
  void uartTransmitThread(void *);
  void gatherLostBytes();
  uint8_t protocol = 0;
  // 6Ko of dma allocated memory
  ObjectFifo<Uart2uavcan_t, 10> *fifoObjectSerial2Uav = nullptr;
  ObjectFifo<Uavcan2uart_t, 50> *fifoObjectUav2Serial = nullptr;
};
