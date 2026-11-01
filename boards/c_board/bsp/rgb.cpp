/*
 * 三色 LED：Timers → TIM5，CH1/CH2/CH3 设为 PWM Generation，引脚 PH10/PH11/PH12 对应蓝/绿/红
 * Prescaler 0，Counter Period 65535，PWM mode 1，CH Polarity High，Pulse 0
 */

#include "bsp/rgb.hpp"
#include "tim.h"
#include <algorithm>

// 以自动重载值为满量程设置通道占空比
static void set_duty(const uint32_t channel, const float duty) {
    const auto period = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(&htim5) + 1);
    __HAL_TIM_SET_COMPARE(&htim5, channel, static_cast<uint32_t>(period * std::clamp(duty, 0.0f, 1.0f)));
}

void rgb_init() {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void Rgb::set(const float red, const float green, const float blue) const {
    set_duty(TIM_CHANNEL_3, red);
    set_duty(TIM_CHANNEL_2, green);
    set_duty(TIM_CHANNEL_1, blue);
}
