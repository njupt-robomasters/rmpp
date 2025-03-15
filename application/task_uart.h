#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void task_uart_entry(void const * argument);

void task_uart_rc_callback(const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif
