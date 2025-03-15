#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void BSP_LED_Init();

void BSP_LED_ShowRGBA(float red, float green, float blue, float alpha);

#ifdef __cplusplus
}
#endif
