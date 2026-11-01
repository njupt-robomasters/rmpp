#include "bsp/irq_lock.hpp"
#include "stm32f4xx_hal.h"

extern void timer_init();
extern void can_init();
extern void uart_init();
extern void cdc_init();
extern void rgb_init();
extern void buzzer_init();
extern void imu_init();

// 由 main.c USER CODE 2 在 CubeMX 外设初始化后调用
extern "C" void bsp_init() {
    timer_init();
    can_init();
    uart_init();
    cdc_init();
    rgb_init();
    buzzer_init();
    imu_init();
}

// 关中断作用域：保存并恢复 PRIMASK，嵌套时只有最外层重新开中断
IrqLock::IrqLock()
    : primask_(__get_PRIMASK()) {
    __disable_irq();
}

IrqLock::~IrqLock() {
    __set_PRIMASK(primask_);
}
