/**
 * @file imavVl53l4cxPort.h
 * @brief Project-owned work-buffer hook for the ST VL53L4CX porting layer.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return DMA-accessible scratch memory owned by the active sensor role.
 *
 * @param device ST driver instance requesting the buffer.
 * @param requiredSize Minimum number of bytes required.
 * @return Buffer address, or NULL if the instance or size is invalid.
 */
uint8_t *imav_vl53l4cx_work_buffer(const void *device,
                                   uint32_t requiredSize);

#ifdef __cplusplus
}
#endif
