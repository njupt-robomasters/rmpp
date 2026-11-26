#pragma once

// 蜂鸣器 PWM-DAC 测试共用：TIM12 CH2 / PB15 80 kHz 载波，采样与载波同步；直接使用 HAL，验证后迁入 bsp

#include "tim.h"
#include <algorithm>
#include <cstdint>

inline constexpr uint32_t PERIOD = 3250;         // 载波 80 kHz
inline constexpr float SAMPLE_DT = 1.0f / 80000; // 采样周期 s，等于载波周期

// 设置占空比，限幅到 [0, 1]
inline void set_duty(const float duty) {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, static_cast<uint32_t>(std::clamp(duty, 0.0f, 1.0f) * PERIOD));
}

// 切换到 80 kHz 载波，占空比 0
inline void start_carrier() {
    __HAL_TIM_SET_PRESCALER(&htim12, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim12, PERIOD - 1);
    set_duty(0);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

// 等待下一个载波周期开始；CCR 预装载，之后写入的值在下一周期生效，每个采样恰好保持一个载波周期
inline void wait_sample() {
    while (!__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE)) {
    }
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);
}

// 占空比在 seconds 秒内线性变化，避免爆音
inline void ramp_duty(const float from, const float to, const float seconds) {
    const int count = static_cast<int>(seconds / SAMPLE_DT);
    for (int i = 1; i <= count; i++) {
        wait_sample();
        set_duty(from + (to - from) * static_cast<float>(i) / static_cast<float>(count));
    }
}

// PolyBLEP 跳变校正：t 为距跳变的相位 [0, 1)，dt 为每采样相位增量
inline float poly_blep(float t, const float dt) {
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1;
    }
    if (t > 1 - dt) {
        t = (t - 1) / dt;
        return t * t + t + t + 1;
    }
    return 0;
}

// 限带方波 ±1：相位 [0, 0.5) 为 1
inline float square_blep(const float phase, const float dt) {
    float half = phase + 0.5f;
    if (half >= 1)
        half -= 1;
    return (phase < 0.5f ? 1.0f : -1.0f) + poly_blep(phase, dt) - poly_blep(half, dt);
}
