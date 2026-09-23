#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "vl53l4cxPort.h"

struct Vl53l4cxState;

/** @brief Independent VL53L4CX distance role, without light/audio dependencies. */
class Vl53l4cxRole final : public RoleBase, public RoleCrtp<Vl53l4cxRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  friend uint8_t *vl53l4cx_work_buffer(const void *, uint32_t);
  static int32_t busInit();
  static int32_t busDeinit();
  static int32_t busWrite(uint16_t address, uint8_t *data, uint16_t length);
  static int32_t busRead(uint16_t address, uint8_t *data, uint16_t length);
  static int32_t getTick();
  msg_t transfer(uint16_t address, const uint8_t *tx, size_t txLength, size_t rxLength);
  bool initialize();
  bool measure();
  void publish(bool valid, float rangeMetres = 0.0f);
  void run(void *);

  Vl53l4cxState *sensor = nullptr;
  uint32_t periodMs = 200U;
  uint8_t sensorId = 0U;
};
