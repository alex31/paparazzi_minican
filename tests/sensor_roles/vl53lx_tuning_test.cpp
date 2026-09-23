#include "vl53lx_api.h"
#include "vl53lx_api_core.h"
#include "vl53lx_preset_setup.h"
#include <cassert>
#include <cstdio>
#include <memory>

// The low-level tuning branch is outside this test. The public API must use
// per-device storage for the bare-driver keys without calling these functions.
extern "C" VL53LX_Error VL53LX_set_tuning_parm(
  VL53LX_DEV, VL53LX_TuningParms, int32_t)
{
  assert(false);
  return VL53LX_ERROR_INVALID_PARAMS;
}
extern "C" VL53LX_Error VL53LX_get_tuning_parm(
  VL53LX_DEV, VL53LX_TuningParms, int32_t *)
{
  assert(false);
  return VL53LX_ERROR_INVALID_PARAMS;
}

int main()
{
  auto first = std::make_unique<VL53L4CX_Object_t>();
  auto second = std::make_unique<VL53L4CX_Object_t>();
  int32_t value = 0;
  assert(VL53LX_GetTuningParameter(first.get(), VL53LX_TUNING_PROXY_MIN, &value)
         == VL53LX_ERROR_NONE);
  assert(value == -30); // Original ST default.
  assert(VL53LX_SetTuningParameter(first.get(), VL53LX_TUNING_PROXY_MIN, -123)
         == VL53LX_ERROR_NONE);
  assert(VL53LX_GetTuningParameter(second.get(), VL53LX_TUNING_PROXY_MIN, &value)
         == VL53LX_ERROR_NONE);
  assert(value == -30); // Changing one context must not change another.
  assert(VL53LX_GetTuningParameter(first.get(), VL53LX_TUNING_PROXY_MIN, &value)
         == VL53LX_ERROR_NONE);
  assert(value == -123);
  assert(VL53LX_GetTuningParameter(first.get(), VL53LX_TUNING_VERSION, &value)
         == VL53LX_ERROR_NONE);
  assert(value == 7);

  // A setter may be the first call on a freshly initialized device.
  auto third = std::make_unique<VL53L4CX_Object_t>();
  assert(VL53LX_SetTuningParameter(third.get(), VL53LX_TUNING_PROXY_MIN, -70)
         == VL53LX_ERROR_NONE);
  assert(VL53LX_GetTuningParameter(third.get(), VL53LX_TUNING_PROXY_MIN, &value)
         == VL53LX_ERROR_NONE);
  assert(value == -70);
  assert(VL53LX_GetTuningParameter(third.get(), VL53LX_TUNING_VERSION, &value)
         == VL53LX_ERROR_NONE);
  assert(value == 7);
  value = 12345;
  assert(VL53LX_GetTuningParameter(first.get(), VL53LX_TUNING_MAX_TUNABLE_KEY, &value)
         == VL53LX_ERROR_INVALID_PARAMS);
  assert(value == 12345);
  assert(VL53LX_SetTuningParameter(first.get(), VL53LX_TUNING_MAX_TUNABLE_KEY, 0)
         == VL53LX_ERROR_INVALID_PARAMS);
  std::puts("VL53L4CX tuning: defaults and per-device isolation OK");
}
