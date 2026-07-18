/**
 * @file imavRole.hpp
 * @brief IMAV beacon sound and light detection role.
 */
#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"

struct ImavAudioState;

/**
 * @brief Detects the acoustic and optical signals emitted by the IMAV beacon.
 *
 * Hardware acquisition and signal processing are introduced incrementally;
 * this first version only integrates the role into the role manager.
 */
class ImavRole final : public RoleBase, public RoleCrtp<ImavRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void audioThread(void * = nullptr);
  void opticalThread(void * = nullptr);
  void audioDmaCallback(ADCDriver *adcp);
  void audioErrorCallback(ADCDriver *adcp, adcerror_t error);
  void startAudioAcquisition();
  void stopAudioAcquisition();
  void processAudioHalf(size_t offset, bool discontinuity);
  void publishDebugValues();
  bool initializeOpticalSensor();
  bool readOpticalSensor();
  bool readOpticalRegister(uint8_t reg, uint16_t& value);
  bool writeOpticalRegister(uint8_t reg, uint16_t value);
  msg_t opticalTransfer(size_t txLength, size_t rxLength);

  ImavAudioState *audio = nullptr;
};
