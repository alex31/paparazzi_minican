/**
 * @file qmc5883Role.hpp
 * @brief QMC5883 magnetometer role definition.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"

/**
 * @brief Role for reading QMC5883 magnetometer data over I2C.
 */
class Qmc5883Role final : public RoleBase, public RoleCrtp<Qmc5883Role> {
public:
  /** @brief Subscribe to UAVCAN messages used by this role. */
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  /** @brief Initialize the sensor and start the polling thread. */
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  /** @brief Periodic polling routine. */
  void periodic(void * = nullptr);

  UAVCAN::Node *m_node = nullptr;
  /** @brief DMA-backed I2C buffers for sensor data. */
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
