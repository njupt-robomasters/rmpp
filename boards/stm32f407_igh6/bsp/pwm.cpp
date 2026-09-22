#include "bsp/pwm.hpp"
#include "peripherals.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>

struct Pwm::State {
    int timer;
    int channel;
    TIM_HandleTypeDef* handle;
    float duty = 0;
};

Pwm::Pwm(int timer, int channel) : timer_(timer), channel_(channel) {
    static constinit State states[] = {
        {.timer = 1, .channel = 1, .handle = &htim1},
        {.timer = 1, .channel = 2, .handle = &htim1},
        {.timer = 1, .channel = 3, .handle = &htim1},
        {.timer = 1, .channel = 4, .handle = &htim1},
        {.timer = 4, .channel = 3, .handle = &htim4},
        {.timer = 5, .channel = 1, .handle = &htim5},
        {.timer = 5, .channel = 2, .handle = &htim5},
        {.timer = 5, .channel = 3, .handle = &htim5},
        {.timer = 8, .channel = 1, .handle = &htim8},
        {.timer = 8, .channel = 2, .handle = &htim8},
        {.timer = 8, .channel = 3, .handle = &htim8},
        {.timer = 10, .channel = 1, .handle = &htim10},
    };
    for (auto& state : states) {
        if (state.timer == timer_ && state.channel == channel_) {
            state_ = &state;
            break;
        }
    }
}

bool Pwm::set_duty(float duty) const {
    if (!state_ || !std::isfinite(duty) || state_->handle->State == HAL_TIM_STATE_RESET)
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    state_->duty = std::clamp(duty, 0.0f, 1.0f);
    const auto count = static_cast<std::uint32_t>((__HAL_TIM_GET_AUTORELOAD(state_->handle) + 1) * state_->duty);
    __HAL_TIM_SET_COMPARE(state_->handle, (state_->channel - 1) * 4, count);
    __set_PRIMASK(primask);
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

    const auto primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_PRESCALER(state_->handle, divisor - 1);
    __HAL_TIM_SET_AUTORELOAD(state_->handle, counts - 1);
    for (int channel = 1; channel <= 4; ++channel) {
        auto* peer = Pwm(state_->timer, channel).state_;
        if (peer)
            __HAL_TIM_SET_COMPARE(peer->handle, (channel - 1) * 4, static_cast<std::uint32_t>(counts * peer->duty));
    }
    // 同时装载预分频和各通道比较值。
    state_->handle->Instance->EGR = TIM_EGR_UG;
    __set_PRIMASK(primask);
    return true;
}

void pwm_init() {
    for (int timer : {1, 4, 5, 8, 10}) {
        for (int channel = 1; channel <= 4; ++channel) {
            auto* state = Pwm(timer, channel).state_;
            if (!state)
                continue;
            // 为 100% 占空比保留 ARR + 1 的可表示比较值。
            if (__HAL_TIM_GET_AUTORELOAD(state->handle) == 65535)
                __HAL_TIM_SET_AUTORELOAD(state->handle, 65534);
            __HAL_TIM_SET_COMPARE(state->handle, (channel - 1) * 4, 0);
            HAL_TIM_PWM_Start(state->handle, (channel - 1) * 4);
        }
    }
}
