#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "microphoneSpectrum.hpp"

struct MicrophoneDmaState;

/** @brief Generic audio spectrum from the analog microphone on PA4/ADC2. */
class MicrophoneRole final : public RoleBase, public RoleCrtp<MicrophoneRole> {
public:
  DeviceStatus subscribe(UAVCAN::Node& node) override;
  DeviceStatus start(UAVCAN::Node& node) override;

private:
  void run(void *);
  void startAcquisition();
  void stopAcquisition();
  void dmaCallback(ADCDriver *);
  void errorCallback(ADCDriver *, adcerror_t error);
  void publish(const MicrophoneSpectrum::Result& spectrum, bool valid);

  MicrophoneDmaState *audio = nullptr;
  MicrophoneSpectrum *spectrum = nullptr;
  thread_t *worker = nullptr;
  sysinterval_t publishPeriod = 0U;
  uint32_t droppedBlocks = 0U;
  uint32_t restarts = 0U;
  uint32_t reportedDroppedBlocks = 0U;
  uint32_t reportedRestarts = 0U;
};
