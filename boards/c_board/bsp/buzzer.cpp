/*
 * 无源蜂鸣器：Timers → TIM4，CH3 设为 PWM Generation，引脚 PD14
 * Prescaler 0，Counter Period 65535，PWM mode 1，CH Polarity High，Pulse 0；运行时按音高重设 PSC/ARR
 */

#include "bsp/buzzer.hpp"
#include "tim.h"
#include <cmath>

void buzzer_init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer::set_tone(const float hz) const {
    // TIM4 位于 APB1，定时器时钟为 HCLK / 2
    const float period = static_cast<float>(SystemCoreClock / 2) / hz;
    const auto psc = static_cast<uint32_t>(std::ceil(period / 65536.0f) - 1);
    const auto arr = static_cast<uint32_t>(std::round(period / static_cast<float>(psc + 1)) - 1);

    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (arr + 1) / 2);
    HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE); // 立即载入新的 PSC
}

void Buzzer::mute() const {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}
