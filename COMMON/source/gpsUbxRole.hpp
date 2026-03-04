/**
 * @file gpsUbxRole.hpp
 * @brief UAVCAN GPS role using UBX frames as input.
 */
#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "gpsUbxDecoder.hpp"
#include "sioWrapper.hpp"


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
   * @brief Acquire resources and start SIO reception.
   */
  DeviceStatus start(UAVCAN::Node& node) override;
  /**
   * @brief Construct the UBX role with its decoder configuration.
   */
  GpsUBX();
  
private:
  /** @brief Max UBX frame size supported by the decoder. */
  static constexpr size_t maxUbxFrameSize = 1024U;
  static constexpr size_t dmaRxSize = maxUbxFrameSize;
  static constexpr size_t dmaRxFifoDepth = 4U;
  static_assert((dmaRxSize % 2U) == 0U, "SIO DMA RX buffer must be even sized");
  using GpsSIO = SIO::Continuous<dmaRxSize, dmaRxFifoDepth>;
  /** @brief SIO RX callback for the UBX decoder. */
  static void sioRxCb(const SIO::ByteSpan &slice, void *user);
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
  /** @brief Continuous SIO driver instance. */
  GpsSIO *sio_ = nullptr;
};
