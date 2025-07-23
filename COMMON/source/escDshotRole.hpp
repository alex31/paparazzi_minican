#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "esc_dshot.h"



class EscDshot : public RoleBase, public RoleCrtp<EscDshot> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void periodic(void *);
  void processEscRawCommand(CanardRxTransfer* transfer, const uavcan_equipment_esc_RawCommand& msg);

  uint8_t mapIndex1 = 0;
  uint8_t numChannels = 1;
  uint16_t rpmFrqDiv = 0;
  sysinterval_t loopPeriod = TIME_MS2I(10);
  uint16_t throttles[4] = {};
  DSHOTDriver dshotd;
  DSHOTConfig dshotConfig;
  DshotDmaBuffer dshotdDmaBuffer;
#if DSHOT_BIDIR
  DshotRpmCaptureDmaBuffer dshotdCaptureDmaBuffer;
#endif
};
