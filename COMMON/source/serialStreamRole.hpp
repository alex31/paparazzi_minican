#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "fifoObject.hpp"


class SerialStream final : public RoleBase, public RoleCrtp<SerialStream> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void processUavcanToSerial(CanardRxTransfer *,
			     const uavcan_tunnel_Broadcast &msg);
  void uartReceiveThread(void *);
  void uavcanTransmitThread(void *);
  void uartTransmitThread(void *);
  uint8_t protocol = 0;
  
};
