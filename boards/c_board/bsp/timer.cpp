/*
 * 64 位周期计数：SysTick 中断把 DWT CYCCNT 的增量累加到 cycles，读取时再补上未累加部分
 * 约束：关中断或在 SysTick 之上的优先级阻塞不得超过一个 CYCCNT 周期（168 MHz 时约 25.5 s）
 */

#include "bsp/timer.hpp"
#include "bsp/irq_lock.hpp"
#include "stm32f4xx_hal.h"
#include <algorithm>

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

float Timer::elapsed() {
    const float seconds = static_cast<float>(read_cycles() - last_cycles_) / static_cast<float>(SystemCoreClock);
    return std::clamp(seconds, min_dt_, max_dt_);
}

float Timer::restart() {
    const uint64_t now = read_cycles();
    const float seconds = static_cast<float>(now - last_cycles_) / static_cast<float>(SystemCoreClock);
    dt = std::clamp(seconds, min_dt_, max_dt_);
    last_cycles_ = now;
    return dt;
}
