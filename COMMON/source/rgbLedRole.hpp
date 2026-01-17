/**
 * @file rgbLedRole.hpp
 * @brief Role for driving an external WS2812 LED strip via UAVCAN.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

/**
 * @brief Role handling UAVCAN LightsCommand for an external LED strip.
 *
 * The role subscribes to uavcan_equipment_indication_LightsCommand and updates a
 * local LED state vector. A background thread periodically pushes that state
 * to a WS2812 strip. The strip length is configured via parameters and the
 * role can be enabled or disabled with "role.led2812".
 */
class RgbLedRole final : public RoleBase, public RoleCrtp<RgbLedRole> {
public:
  /** @brief Subscribe to the required UAVCAN message types. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Acquire hardware resources and start the LED update thread. */
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  /** @brief Handle incoming LightsCommand messages. */
  void processLightsCommand(CanardRxTransfer *,
			    const  uavcan_equipment_indication_LightsCommand &msg);

};
