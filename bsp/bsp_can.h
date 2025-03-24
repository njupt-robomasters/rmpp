#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*CAN_Callback_t)(uint32_t id, uint8_t data[8]);

void BSP_CAN_Init();

void BSP_CAN_Transmit(uint32_t id, uint8_t data[8], uint8_t dlc);

void BSP_CAN_SetCallback(CAN_Callback_t callback);

#ifdef __cplusplus
}
#endif
