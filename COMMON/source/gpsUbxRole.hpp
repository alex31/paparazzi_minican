#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "gpsUbxDecoder.hpp"
#include "etl/span.h"
#include <cstddef>


class GpsUBX final : public RoleBase, public RoleCrtp<GpsUBX> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;
  GpsUBX();
  
private:
  static constexpr size_t maxUbxFrameSize = 640U;
  static bool navPvtCb(const UBX::NavPvt& msg);
  static bool navDopCb(const UBX::NavDop& msg);
  static bool navSatCb(const UBX::NavSat& msg);
  inline static constexpr UBX::DecoderConf config = {navPvtCb, navDopCb, navSatCb};
  UBX::Decoder decoder;
  struct DopCache {
    float gdop = 0.F;
    float pdop = 0.F;
    float tdop = 0.F;
    float vdop = 0.F;
    float hdop = 0.F;
    float ndop = 0.F;
    float edop = 0.F;
    bool  valid = false;
  } dopCache;
  uint8_t satsVisible = 0;
  uint8_t satsUsed = 0;
  uint8_t *frame;
  void periodic(void *);
};
