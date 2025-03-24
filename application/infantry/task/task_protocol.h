#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void task_protocol_rv2_entry(void const *argument);

[[noreturn]] void task_protocol_ui_entry(void const *argument);

void task_protocol_rc_callback(const uint8_t *data, uint16_t size);

void task_protocol_referee_callback(const uint8_t *data, uint16_t size);

void task_protocol_cdc_callback(const uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif
