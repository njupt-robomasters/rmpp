#include "board/pwm.hpp"
#include "irq_lock.hpp"
#include "peripherals.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>

namespace rmpp {
struct Pwm::State {
    int timer;
    int channel;
    TIM_HandleTypeDef* handle;
    float duty = 0;
};

Pwm::State* Pwm::find(int timer, int channel) {
    static State states[] = {
        {1, 1, &htim1},
        {1, 2, &htim1},
        {1, 3, &htim1},
        {1, 4, &htim1},
        {4, 3, &htim4},
        {5, 1, &htim5},
        {5, 2, &htim5},
        {5, 3, &htim5},
        {8, 1, &htim8},
        {8, 2, &htim8},
        {8, 3, &htim8},
        {10, 1, &htim10},
    };
    for (auto& state : states) {
        if (state.timer == timer && state.channel == channel)
            return &state;
    }
    return nullptr;
}

bool Pwm::set_duty(float duty) const {
    if (!state_ || !std::isfinite(duty) || state_->handle->State == HAL_TIM_STATE_RESET)
        return false;
    IrqLock lock;
    state_->duty = std::clamp(duty, 0.0f, 1.0f);
    const auto count = static_cast<std::uint32_t>((__HAL_TIM_GET_AUTORELOAD(state_->handle) + 1) * state_->duty);
    __HAL_TIM_SET_COMPARE(state_->handle, (state_->channel - 1) * 4, count);
    return true;
}

bool Pwm::set_frequency(float hz) const {
    if (!state_ || !std::isfinite(hz) || hz <= 0 || state_->handle->State == HAL_TIM_STATE_RESET)
        return false;
    RCC_ClkInitTypeDef clocks{};
    std::uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const bool apb2 = state_->timer == 1 || state_->timer == 8 || state_->timer == 10;
    const auto divider = apb2 ? clocks.APB2CLKDivider : clocks.APB1CLKDivider;
    const auto clock = (apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq()) * (divider == RCC_HCLK_DIV1 ? 1u : 2u);
    const double period = static_cast<double>(clock) / hz;
    const double prescaler = std::ceil(period / 65535.0);
    if (period < 1 || prescaler > 65536)
        return false;
    const auto divisor = static_cast<std::uint32_t>(std::max(prescaler, 1.0));
    const auto counts = static_cast<std::uint32_t>(std::round(period / divisor));
    if (counts == 0 || counts > 65535)
        return false;

    IrqLock lock;
    __HAL_TIM_SET_PRESCALER(state_->handle, divisor - 1);
    __HAL_TIM_SET_AUTORELOAD(state_->handle, counts - 1);
    for (int channel = 1; channel <= 4; ++channel) {
        auto* peer = find(state_->timer, channel);
        if (peer)
            __HAL_TIM_SET_COMPARE(peer->handle, (channel - 1) * 4, static_cast<std::uint32_t>(counts * peer->duty));
    }
    // Transfer the prescaler and preloaded compare values together.
    state_->handle->Instance->EGR = TIM_EGR_UG;
    return true;
}

void pwm_init() {
    for (int timer : {1, 4, 5, 8, 10}) {
        for (int channel = 1; channel <= 4; ++channel) {
            auto* state = Pwm::find(timer, channel);
            if (!state)
                continue;
            // Leave one representable compare value above ARR for 100% duty.
            if (__HAL_TIM_GET_AUTORELOAD(state->handle) == 65535)
                __HAL_TIM_SET_AUTORELOAD(state->handle, 65534);
            __HAL_TIM_SET_COMPARE(state->handle, (channel - 1) * 4, 0);
            HAL_TIM_PWM_Start(state->handle, (channel - 1) * 4);
        }
    }
}
} // namespace rmpp
