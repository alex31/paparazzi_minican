/**
 * @file gpsUbxRole.hpp
 * @brief UAVCAN GPS role using UBX frames as input.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"
#include "gpsUbxDecoder.hpp"
#include "etl/span.h"
#include <cstddef>


/**
 * @brief GPS role that decodes UBX NAV messages and publishes DroneCAN GNSS.
 */
class GpsUBX final : public RoleBase, public RoleCrtp<GpsUBX> {
public:
  /**
   * @brief Advertise role capabilities (no subscriptions required).
   */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /**
   * @brief Acquire resources, start UART, and spawn decoder thread.
   */
  DeviceStatus start(UAVCAN::Node& node) override;
  /**
   * @brief Construct the UBX role with its decoder configuration.
   */
  GpsUBX();
  
private:
  /** @brief Max UBX frame size supported by the decoder. */
  static constexpr size_t maxUbxFrameSize = 1024U;
  /** @brief UBX NAV-PVT handler (Fix2 + status publishing). */
  static bool navPvtCb(const UBX::NavPvt& msg);
  /** @brief UBX NAV-DOP handler (Auxiliary publication). */
  static bool navDopCb(const UBX::NavDop& msg);
  /** @brief UBX NAV-SAT handler (satellite count aggregation). */
  static bool navSatCb(const UBX::NavSat& msg);
  /** @brief Decoder configuration with static callbacks. */
  inline static constexpr UBX::DecoderConf config = {navPvtCb, navDopCb, navSatCb};
  /** @brief UBX decoder instance. */
  UBX::Decoder decoder;
  /** @brief Cached DOP values for auxiliary publishing. */
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
  /** @brief Running count of visible satellites for the last NAV-SAT epoch. */
  uint8_t satsVisible = 0;
  /** @brief Satellites used as reported by NAV-PVT. */
  uint8_t satsUsed = 0;
  /** @brief Last NAV-SAT iTOW seen (used to reset visibility count). */
  uint32_t lastNavSatITOW = 0;
  /** @brief DMA buffer used for UART reception. */
  uint8_t *frame;
  /**
   * @brief UART RX thread body, feeds the UBX decoder.
   */
  void periodic(void *);
};
