#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Per-role scratch space for the unchanged ST component driver. */
uint8_t *vl53l4cx_work_buffer(const void *device, uint32_t required_size);
void vl53l4cx_sleep_us(uint32_t microseconds);

#ifdef __cplusplus
}
#endif
