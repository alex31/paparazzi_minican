#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "opt4060Sample.hpp"
#include "opt4060Timing.hpp"
#include "lightPublication.hpp"

/** @brief Independent RGBW acquisition, with CRC and optional PA8 data-ready. */
class Opt4060Role final : public RoleBase, public RoleCrtp<Opt4060Role> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  struct DmaBuffers {
    uint8_t tx[3];
    uint8_t rx[16];
  };
  msg_t transfer(size_t txLength, size_t rxLength);
  bool readRegister(uint8_t reg, uint16_t& value);
  bool writeRegister(uint8_t reg, uint16_t value);
  bool initialize();
  Opt4060Sample::Result readSample();
  void run(void *);
  void observe(bool decoded, uint64_t timestampUs);

  DmaBuffers *dma = nullptr;
  Opt4060Sample::Frame frame;
  Opt4060Timing::Settings timing = {};
  LightPublication publication;
  uint8_t address = 0x44U;
  uint8_t sensorId = 0U;
  bool useInterrupt = true;
  bool haveFrame = false;
  bool overloaded = false;
};
