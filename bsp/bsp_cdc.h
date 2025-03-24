#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*CDC_Callback_t)(const uint8_t *data, uint32_t size);

void BSP_CDC_Init();

void BSP_CDC_Transmit(uint8_t *data, uint16_t size);

void BSP_CDC_printf(const char *format, ...);

void BSP_CDC_SetCallback(CDC_Callback_t callback);

void BSP_CDC_Callback_ForHAL(const uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif
