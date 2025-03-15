#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_CDC_Init();

void BSP_CDC_Transmit(uint8_t *data, int size);

void BSP_CDC_printf(const char *format, ...);

void BSP_CDC_Rx_Callback(const uint8_t *data, int size);

#ifdef __cplusplus
}
#endif
