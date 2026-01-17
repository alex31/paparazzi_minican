/**
 * @file mfsOnM95p.h
 * @brief C interface for MFS storage on STM M95P EEPROM.
 */
#pragma once
#include "hal_xsnor_stm_m95p.h"

#ifdef __cplusplus
extern "C" {
#endif

  /** @brief Initialize the M95P-backed MFS storage. */
  bool mfsOverM95pInit(void);
  /** @brief Retrieve the underlying M95P device handle. */
  hal_xsnor_stm_m95p_c* mfsOverM95pGetDevice(void);
  
#ifdef __cplusplus
}
#endif
