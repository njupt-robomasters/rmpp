#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_CAN_Init();

void BSP_CAN_Transmit(uint32_t id, uint8_t data[8], uint8_t dlc);

void BSP_CAN_Rx_Callback(uint32_t id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif
