#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"


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
  bool pushToFifo(input_queue_t &fifo, const uint8_t *data, size_t len);
  uint8_t protocol = 0;
  uint8_t *recBuffer = nullptr;
  uint8_t *txBuffer = nullptr;
  uint8_t *uartToCanBuffer = nullptr;
  uint8_t *canToUartBuffer = nullptr;
  input_queue_t uartToCanFifo;
  input_queue_t canToUartFifo;
};
