#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"

class Qmc5883Role final : public RoleBase, public RoleCrtp<Qmc5883Role> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void periodic(void * = nullptr);

  UAVCAN::Node *m_node = nullptr;
  struct DmaBuffers {
    int16_t axes[3];
    uint8_t ctrl1[2];
    uint8_t status;
  };
  DmaBuffers *dmaBuf = nullptr;
  float countsPerGauss = 3000.0f; // default 8G sensitivity
  uint16_t rotDeg = 0;
  uint8_t sensorId = 0;
};
