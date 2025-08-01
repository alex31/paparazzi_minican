#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleStatus.hpp"
#include "roleBase.hpp"

class Baro_MPL3115A2_Role final : public RoleBase, public RoleCrtp<Baro_MPL3115A2_Role> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;
  DeviceStatus getDevId(uint8_t *devId);
  DeviceStatus getPressure(float *pressure);
  DeviceStatus printPressure();
  DeviceStatus getTemperature(float *temperature);

private:
  void periodic();
  void resetI2C();
};
