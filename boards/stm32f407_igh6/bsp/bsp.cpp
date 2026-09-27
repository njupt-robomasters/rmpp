#include "bsp/bsp.hpp"

extern "C" void bsp_init() {
    timer_init();
    pwm_init();
    can_init();
    uart_init();
}
