#include "bsp/timer.hpp"
#include "bsp/timer.h"
#include "peripherals.hpp"
#include "stm32f4xx.h"
#include <algorithm>
#include <cmath>

void timer_init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

float Timer::get_dt() {
    if (!has_baseline_)
        reset();
    dt = timer_get_dt(last_tick_);
    if (dt > max_dt_)
        at_limit_ = true;
    dt = at_limit_ ? max_dt_ : std::max(dt, min_dt_);
    return dt;
}

float Timer::update_dt() {
    get_dt();
    last_tick_ = DWT->CYCCNT;
    at_limit_ = false;
    freq = dt > 0 ? 1.0f / dt : 0;
    return dt;
}

bool Timer::poll_timeout(float seconds) {
    if (get_dt() < seconds)
        return false;
    update_dt();
    return true;
}

void Timer::reset() {
    last_tick_ = DWT->CYCCNT;
    has_baseline_ = true;
    at_limit_ = false;
}

void Timer::delay(float seconds) {
    if (!std::isfinite(seconds) || seconds <= 0)
        return;
    // Short chunks keep the cycle difference valid across counter wraparound.
    while (seconds > 0) {
        const float chunk = std::min(seconds, 1.0f);
        const auto ticks = static_cast<std::uint32_t>(chunk * SystemCoreClock);
        const auto begin = DWT->CYCCNT;
        while (static_cast<std::uint32_t>(DWT->CYCCNT - begin) < ticks) {
        }
        seconds -= chunk;
    }
}

extern "C" float timer_get_dt(uint32_t last_tick) {
    return static_cast<float>(static_cast<uint32_t>(DWT->CYCCNT - last_tick)) / SystemCoreClock;
}

extern "C" float timer_update_dt(uint32_t* last_tick) {
    const auto now = DWT->CYCCNT;
    const float dt = static_cast<float>(static_cast<uint32_t>(now - *last_tick)) / SystemCoreClock;
    *last_tick = now;
    return dt;
}

extern "C" void timer_delay(float seconds) {
    Timer::delay(seconds);
}
