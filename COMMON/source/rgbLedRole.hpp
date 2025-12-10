#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

/*
  The role subscribe to uavcan_equipment_indication_SingleLightCommand
  it manage a led 2812 strip using led2812.hpp module
  it read UAVCan parameter "ROLE.led2812" to enable disable the role
  "role.led2812.led_number" to specify the maximum number of led : 1 .. 8
  led2812.hpp module usage can be studied by looking at rgbLeds.hpp rbgLeds.cpp
  which use the same kind of led that is welded on the minican to display node state
  This module is thre to manage external strip of 1 to 8 leds on the airframe
  to simplify, we assume that there is only one led strip on the airframe so
  we map directly uavcan message led index to les index in the led strip.
  We use only PB07 : TIM3_CH4 on both minican or microcan to drive the ledstrip
  ressource must be taken : TIM3 is not yet a managed ressource, so it must be added to the
  ressource module. On minican, and microcan, the PB07 pin should be affected to TIM3_CH4 with
  palSetLineMode.
  Example should be taken from the other roles to see what to do.
  For this role, a thread should be launched that will send periodicly the led pattern

  In the callback of the message, we just update a state vactor, this vector of values is used by
  the thread to send perodicly the values to the led strip

  the rgbLedRole.cpp should be completed
 */


class RgbLedRole final : public RoleBase, public RoleCrtp<RgbLedRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void processLightsCommand(CanardRxTransfer *,
			    const  uavcan_equipment_indication_LightsCommand &msg);

};
