/*
 * 无源蜂鸣器：Timers → TIM4，CH3 设为 PWM Generation，引脚 PD14
 * Prescaler 0，Counter Period 65535，PWM mode 1，CH Polarity High，Pulse 0；运行时按音高重设 PSC/ARR
 *
 * 采样流：DMA → DMA1 添加 TIM4_UP：Stream6 Channel 2，Memory To Peripheral，Circular，只递增 Memory，Half Word，不开 FIFO
 * NVIC DMA1 stream6 优先级 15
 */

#include "bsp/buzzer.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>

static constexpr float FILL_PERIOD = 0.001f; // 填充周期 s，半区采样数按载波换算
static constexpr std::size_t MAX_HALF = 64;  // 半区采样数上限，载波最高 64 kHz

static uint16_t buffer[2 * MAX_HALF];
static Buzzer::Fill stream_fill;
static uint32_t stream_period = 0; // ARR + 1
static std::size_t half = 0;       // 本次采样流的半区采样数

// 定时器时钟：TIM4 位于 APB1，为 HCLK / 2
static float timer_clock() {
    return static_cast<float>(SystemCoreClock / 2);
}

// 向 fill 要一块占空比并换算为 CCR 写入半区
static void fill_half(uint16_t* data) {
    float duty[MAX_HALF];
    stream_fill(std::span(duty, half));
    for (std::size_t i = 0; i < half; i++)
        data[i] = static_cast<uint16_t>(std::clamp(duty[i], 0.0f, 1.0f) * static_cast<float>(stream_period));
}

void buzzer_init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer::set_tone(const float hz) const {
    const float period = timer_clock() / hz;
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

void Buzzer::start_stream(const float carrier_hz, Fill fill) const {
    // 载波：PSC 0，ARR 按载波频率；CCR 预装载，DMA 写入的值在下一载波周期生效
    stream_period = static_cast<uint32_t>(std::lround(timer_clock() / carrier_hz));
    __HAL_TIM_SET_PRESCALER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, stream_period - 1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);

    // 半区按载波换算，半满、全满中断约每 FILL_PERIOD 一次
    half = std::clamp<std::size_t>(std::lround(carrier_hz * FILL_PERIOD), 1, MAX_HALF);

    // 先填满两个半区，之后在半满、全满中断里填刚送完的半区
    stream_fill = std::move(fill);
    fill_half(buffer);
    fill_half(buffer + half);
    DMA_HandleTypeDef* const hdma = htim4.hdma[TIM_DMA_ID_UPDATE];
    hdma->XferHalfCpltCallback = [](DMA_HandleTypeDef*) { fill_half(buffer); };
    hdma->XferCpltCallback = [](DMA_HandleTypeDef*) { fill_half(buffer + half); };
    HAL_DMA_Start_IT(hdma, reinterpret_cast<uint32_t>(buffer), reinterpret_cast<uint32_t>(&TIM4->CCR3), 2 * half);
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_UPDATE);
}

void Buzzer::stop_stream() const {
    __HAL_TIM_DISABLE_DMA(&htim4, TIM_DMA_UPDATE);
    HAL_DMA_Abort(htim4.hdma[TIM_DMA_ID_UPDATE]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}
