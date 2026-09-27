#include "bsp/timer.hpp"
#include "stm32f4xx_hal.h"
#include <algorithm>
#include <cmath>

static constexpr uint32_t DWT_MAX_ELAPSED_MS = 10'000;

void timer_init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Timer::delay(const float seconds) {
    if (!std::isfinite(seconds) || seconds <= 0)
        return;

    const float delay_ms = seconds * 1000.0f;
    if (delay_ms <= static_cast<float>(DWT_MAX_ELAPSED_MS)) {
        const auto ticks = static_cast<uint32_t>(seconds * static_cast<float>(SystemCoreClock));
        const auto begin = DWT->CYCCNT;
        while (DWT->CYCCNT - begin < ticks) {
        }
    } else {
        const auto begin = HAL_GetTick();
        while (static_cast<float>(HAL_GetTick() - begin) < delay_ms) {
        }
    }
}

float Timer::elapsed() {
    const auto now_tick = DWT->CYCCNT;
    const auto elapsed_ticks = now_tick - last_tick_;

    const auto now_ms = HAL_GetTick();
    const auto elapsed_ms = now_ms - last_ms_;

    float seconds;
    if (elapsed_ms <= DWT_MAX_ELAPSED_MS)
        seconds = static_cast<float>(elapsed_ticks) / static_cast<float>(SystemCoreClock);
    else
        seconds = static_cast<float>(elapsed_ms) / 1000.0f;

    dt = std::clamp(seconds, min_dt_, max_dt_);

    return dt;
}

float Timer::restart() {
    const float result = elapsed();
    last_tick_ = DWT->CYCCNT;
    last_ms_ = HAL_GetTick();
    return result;
}
