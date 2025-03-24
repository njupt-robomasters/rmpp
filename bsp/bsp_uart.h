#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UART_Callback_t)(const uint8_t *data, uint16_t size);

void BSP_UART_Init();

void BSP_UART_Referee_Video_Transmit(const uint8_t *data,  uint16_t size);

void BSP_UART_RC_SetCallback(UART_Callback_t callback);

void BSP_UART_Referee_SetCallback(UART_Callback_t callback);

#ifdef __cplusplus
}
#endif
