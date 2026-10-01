extern void timer_init();
extern void pwm_init();
extern void can_init();
extern void uart_init();

extern "C" void bsp_init() {
    timer_init();
    pwm_init();
    can_init();
    uart_init();
}
