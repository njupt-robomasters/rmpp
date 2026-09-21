#include "bsp.hpp"
#include "peripherals.hpp"

void bsp_init() {
    rmpp::timer_init();
    rmpp::pwm_init();
    rmpp::can_init();
    rmpp::uart_init();
}
