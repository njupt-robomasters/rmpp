#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void task_cdc_entry(void const * argument);

void task_cdc_callback(const uint8_t *data, int size);

#ifdef __cplusplus
}
#endif
