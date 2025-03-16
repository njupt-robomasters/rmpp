#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_DWT_Init();

float BSP_DWT_GetDeltaT(uint32_t *last_tick);

double BSP_DWT_GetDeltaT64(uint32_t *last_tick);

float BSP_DWT_GetTime_second();

float BSP_DWT_GetTime_ms();

float BSP_DWT_GetTime_us();

void BSP_DWT_Delay(float second);

#ifdef __cplusplus
}
#endif
