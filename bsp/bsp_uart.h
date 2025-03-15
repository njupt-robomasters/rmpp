#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_UART_RC_Init();

void BSP_UART_RC_Rx_Callback(const uint8_t *data, uint16_t size);

void BSP_UART_Referee_Rx_Callback(const uint8_t *data, uint16_t size);

void BSP_UART_Video_Rx_Callback(const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif
