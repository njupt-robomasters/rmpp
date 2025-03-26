#include "task_protocol.h"
#include "app.hpp"

[[noreturn]] void task_protocol_nav_entry(void const *argument) {
    while (true) {
        nav.SendTargetData(status.nav_target_x, status.nav_target_y);
        osDelay(100);
    }
}

void task_protocol_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}

void task_protocol_referee_callback(const uint8_t *data, const uint16_t size) {
    referee.PhaseData(data, size);
}

void task_protocol_cdc_callback(const uint8_t *data, const uint32_t size) {
    nav.ParseStreamingData(data, size);
}
