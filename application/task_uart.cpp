#include "task_uart.h"
#include "app_variable.hpp"

[[noreturn]] void task_uart_entry(void const * argument) {
    while (true) {
        osDelay(1000);
    }
}

void task_uart_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}
