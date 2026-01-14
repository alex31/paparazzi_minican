#pragma once

namespace Adc {
  using Callback_t = void  (float psBat, float coreTemp);
  void start(Callback_t *cb = nullptr);
  void setErrorCB(Callback_t *cb);
  float getPsBatRaw();
  float getPsBat();
  float getCoreTemp();
  float getVcc();
}
