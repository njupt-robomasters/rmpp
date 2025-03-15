#pragma once

#include <stdint.h>

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

#ifdef __cplusplus
extern "C" {
#endif

extern DWT_Time_t SysTime;

void BSP_DWT_Init();

float BSP_DWT_GetDeltaT(uint32_t *cnt_last);

double BSP_DWT_GetDeltaT64(uint32_t *cnt_last);

float BSP_DWT_GetTimeline_s();

float BSP_DWT_GetTimeline_ms();

uint64_t BSP_DWT_GetTimeline_us();

void BSP_DWT_Delay(float Delay);

void DWT_SysTimeUpdate();

#ifdef __cplusplus
}
#endif
