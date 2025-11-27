#pragma once

namespace Adc {
  using Callback_t = void  (float psBat, float coreTemp);
  void start(Callback_t *cb);
#if PLATFORM_MICROCAN
  uint8_t getAddress();
#endif
  float getPsBat();
  float getCoreTemp();
  float getVcc();
}

