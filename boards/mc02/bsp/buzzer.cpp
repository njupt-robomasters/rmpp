/*
 * 无源蜂鸣器：Timers → TIM12，CH2 设为 PWM Generation，引脚 PB15
 * Prescaler 0，Counter Period 65535，PWM mode 1，CH Polarity High，Pulse 0；TRGO 选 Update Event
 *
 * 采样流：TIM12 无 DMA 请求，由 DMAMUX 请求发生器以 TIM12_TRGO 触发
 * DMA → DMA2 添加 DMA_GENERATOR0：Stream0，Memory To Peripheral，Circular，只递增 Memory，Half Word，不开 FIFO
 * Request Generator：Signal TIM12_TRGO，Rising，Request Number 1；NVIC DMA2 stream0 优先级 15
 * 双缓冲放 .dma_buffer（RAM_D1），填完半区后清理 D-Cache
 */

#include "bsp/buzzer.hpp"
#include "dma.h"
#include "tim.h"
#include <algorithm>
#include <cmath>

static constexpr float FILL_PERIOD = 0.001f; // 填充周期 s，半区采样数按载波换算
static constexpr std::size_t MAX_HALF = 64;  // 半区采样数上限，载波最高 64 kHz

__attribute__((section(".dma_buffer"))) alignas(32) static uint16_t buffer[2 * MAX_HALF];
static Buzzer::Fill stream_fill;
static uint32_t stream_period = 0; // ARR + 1
static std::size_t half = 0;       // 本次采样流的半区采样数

// 定时器时钟：TIM12 位于 APB1，分频不为 1 时为 PCLK1 × 2
static float timer_clock() {
    return static_cast<float>(HAL_RCC_GetPCLK1Freq() * 2);
}

// 向 fill 要一块占空比并换算为 CCR 写入半区
// Clean 按缓存行覆盖半区，半区长度不必对齐缓存行
static void fill_half(uint16_t* data) {
    float duty[MAX_HALF];
    stream_fill(std::span(duty, half));
    for (std::size_t i = 0; i < half; i++)
        data[i] = static_cast<uint16_t>(std::clamp(duty[i], 0.0f, 1.0f) * static_cast<float>(stream_period));
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(data), static_cast<int32_t>(sizeof(uint16_t) * half));
}

void buzzer_init() {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Buzzer::set_tone(const float hz) const {
    const float period = timer_clock() / hz;
    const auto psc = static_cast<uint32_t>(std::ceil(period / 65536.0f) - 1);
    const auto arr = static_cast<uint32_t>(std::round(period / static_cast<float>(psc + 1)) - 1);

    __HAL_TIM_SET_PRESCALER(&htim12, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (arr + 1) / 2);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE); // 立即载入新的 PSC
}

void Buzzer::mute() const {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}

void Buzzer::start_stream(const float carrier_hz, Fill fill) const {
    // 载波：PSC 0，ARR 按载波频率；CCR 预装载，DMA 写入的值在下一载波周期生效
    stream_period = static_cast<uint32_t>(std::lround(timer_clock() / carrier_hz));
    __HAL_TIM_SET_PRESCALER(&htim12, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim12, stream_period - 1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);

    // 半区按载波换算，半满、全满中断约每 FILL_PERIOD 一次
    half = std::clamp<std::size_t>(std::lround(carrier_hz * FILL_PERIOD), 1, MAX_HALF);

    // 先填满两个半区，之后在半满、全满中断里填刚送完的半区
    stream_fill = std::move(fill);
    fill_half(buffer);
    fill_half(buffer + half);
    hdma_dma_generator0.XferHalfCpltCallback = [](DMA_HandleTypeDef*) { fill_half(buffer); };
    hdma_dma_generator0.XferCpltCallback = [](DMA_HandleTypeDef*) { fill_half(buffer + half); };
    HAL_DMA_Start_IT(&hdma_dma_generator0, reinterpret_cast<uint32_t>(buffer), reinterpret_cast<uint32_t>(&TIM12->CCR2), 2 * half);
    HAL_DMAEx_EnableMuxRequestGenerator(&hdma_dma_generator0);
}

void Buzzer::stop_stream() const {
    HAL_DMAEx_DisableMuxRequestGenerator(&hdma_dma_generator0);
    HAL_DMA_Abort(&hdma_dma_generator0);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}
