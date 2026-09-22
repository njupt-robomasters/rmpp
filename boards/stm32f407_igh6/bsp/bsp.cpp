#include "bsp.hpp"
#include "peripherals.hpp"

void bsp_init() {
    timer_init();
    pwm_init();
    can_init();
    uart_init();
}
