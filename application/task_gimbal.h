#pragma once

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void task_gimbal_entry(void const *argument);

void task_gimbal_callback(uint32_t id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif
