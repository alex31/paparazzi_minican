#pragma once
#include "hal_xsnor_stm_m95p.h"

#ifdef __cplusplus
extern "C" {
#endif

  bool mfsOverM95pInit(void);
  hal_xsnor_stm_m95p_c* mfsOverM95pGetDevice(void);
  
#ifdef __cplusplus
}
#endif
