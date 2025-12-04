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
  };


  void processUavcanToSerial(CanardRxTransfer *,
			     const uavcan_tunnel_Broadcast &msg);
  void uartReceiveThread(void *);
  void uavcanTransmitThread(void *);
  void uartTransmitThread(void *);
  uint8_t protocol = 0;
  static ObjectFifo<Uart2uavcan_t, 5> fifoObjectSerial2Uav;
  static ObjectFifo<Uavcan2uart_t, 25> fifoObjectUav2Serial;
};
