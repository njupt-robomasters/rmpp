#pragma once

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void task_chassis_entry(void *argument);

void task_chassis_callback(uint32_t id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif
