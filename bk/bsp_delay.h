#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void delay_ticks(uint32_t ticks);

void delay_us(uint16_t nus);

void delay_ms(uint16_t nms);

void delay(uint16_t nms);

#ifdef __cplusplus
}
#endif
