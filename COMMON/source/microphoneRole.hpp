#pragma once

#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "microphoneStatistics.hpp"

struct MicrophoneDmaState;

/** @brief Independent IM68A130 analog microphone acquisition on PA4/ADC2. */
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
  void publish(const MicrophoneStatistics& statistics, bool valid);

  MicrophoneDmaState *audio = nullptr;
  thread_t *worker = nullptr;
  sysinterval_t publishPeriod = 0U;
  uint32_t droppedBlocks = 0U;
  uint32_t restarts = 0U;
};
