#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "pprzLink.hpp"


class TelemetryTunnel final : public RoleBase, public RoleCrtp<TelemetryTunnel> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void processPaparazziTelemetryCommand_u2s(CanardRxTransfer *,
					    const  paparazzi_tunnel_Telemetry &msg);
  void processPaparazziTelemetryCommand_s2u(PprzPolicy pol,
					    std::span<const uint8_t> msg);
  static void trapError_s2u(uint32_t v, uint32_t i);
 
  void periodic(void *);

  static bool xbeeFrame;

  // we use one or the other depending on a node parameter
  using XBEEBufferType = PprzDecoder<
    PprzPolicy::XBEE_API,
    500_ms,
    Trampoline<&TelemetryTunnel::processPaparazziTelemetryCommand_s2u>::fn,
    trapError_s2u
    >;
  
  using PPRZBufferType = PprzDecoder<
    PprzPolicy::PPRZ,
    500_ms,
    Trampoline<&TelemetryTunnel::processPaparazziTelemetryCommand_s2u>::fn,
    trapError_s2u
    >;

  union {
    struct {
      PPRZBufferType *ppS2UBuffer = nullptr;
      PprzMsg_t *pprzMsg = nullptr;
    };
    struct {
      XBEEBufferType *xbS2UBuffer;
      XbeeMsg_t *xbeeMsg;
    };
  };
};
