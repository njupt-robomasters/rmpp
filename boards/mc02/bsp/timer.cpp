/*
 * 64 位周期计数：SysTick 中断把 DWT CYCCNT 的增量累加到 cycles，读取时再补上未累加部分
 * 约束：关中断或在 SysTick 之上的优先级阻塞不得超过一个 CYCCNT 周期（520 MHz 时约 8.26 s）
 */

#include "bsp/timer.hpp"
#include "bsp/irq_lock.hpp"
#include "stm32h7xx_hal.h"

static volatile uint64_t cycles = 0;      // 截至上次累加的周期数
static volatile uint32_t last_cyccnt = 0; // 上次累加时的 CYCCNT

// 覆盖 HAL 弱定义，由 SysTick 中断调用
extern "C" void HAL_IncTick() {
    uwTick += uwTickFreq;

    const uint32_t now = DWT->CYCCNT;
    cycles = cycles + (now - last_cyccnt);
    last_cyccnt = now;
}

// 当前周期数，中断内可调用
static uint64_t read_cycles() {
    const IrqLock lock;
    return cycles + (DWT->CYCCNT - last_cyccnt);
}

void timer_init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55; // Cortex-M7 需先解锁 DWT 寄存器

    const IrqLock lock;
    DWT->CYCCNT = 0;
    last_cyccnt = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Timer::delay(const float seconds) {
    const uint64_t end = read_cycles() + static_cast<uint64_t>(seconds * static_cast<float>(SystemCoreClock));
    while (read_cycles() < end) {
    }
}

float Timer::elapsed() const {
    return static_cast<float>(read_cycles() - last_cycles) / static_cast<float>(SystemCoreClock);
}

float Timer::restart() {
    const uint64_t now = read_cycles();
    const float seconds = static_cast<float>(now - last_cycles) / static_cast<float>(SystemCoreClock);
    last_cycles = now;
    return seconds;
}
