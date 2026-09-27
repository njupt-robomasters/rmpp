/*
 * CubeMX 配置：Timers → 对应 TIM
 *
 * Mode 中将下列 Channel 设为 PWM Generation CHx，Pinout 选择对应引脚
 * | TIM   | Channel               | 引脚（按通道顺序）       | Prescaler | Counter Period |
 * | ----- | --------------------- | ------------------------ | --------- | -------------- |
 * | TIM1  | CH1 / CH2 / CH3 / CH4 | PE9 / PE11 / PE13 / PE14 | 0         | 65535          |
 * | TIM4  | CH3                   | PD14                     | 0         | 65535          |
 * | TIM5  | CH1 / CH2 / CH3       | PH10 / PH11 / PH12       | 0         | 65535          |
 * | TIM8  | CH1 / CH2 / CH3       | PC6 / PI6 / PI7          | 0         | 65535          |
 * | TIM10 | CH1                   | PF6                      | 167       | 999            |
 *
 * 通用参数
 * | 页面                                | 参数                               | 设置                         |
 * | ----------------------------------- | ---------------------------------- | ---------------------------- |
 * | Mode                                | Clock Source                       | Internal Clock（有此选项时） |
 * | Parameter Settings                  | Counter Mode / auto-reload preload | Up / Disable                 |
 * | Parameter Settings → PWM Generation | Mode / CH Polarity / Pulse         | PWM mode 1 / High / 0        |
 *
 * 启动 PWM 和运行时调频由 BSP 完成；TIM10 上电频率为 1 kHz
 */

#include "bsp/pwm.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

struct Pwm::Context {
    int timer;
    int channel;
    TIM_HandleTypeDef* htim;
    float duty = 0;
};

void pwm_init() {
    for (const int timer : {1, 4, 5, 8, 10}) {
        for (int channel = 1; channel <= 4; channel++) {
            if (const auto* context = Pwm(timer, channel).context_) {
                HAL_TIM_PWM_Start(context->htim, (channel - 1) * 4);
            }
        }
    }
}

Pwm::Pwm(const int timer, const int c)  {
    static constinit Context contexts[] = {
        {.timer = 1, .channel = 1, .htim = &htim1},
        {.timer = 1, .channel = 2, .htim = &htim1},
        {.timer = 1, .channel = 3, .htim = &htim1},
        {.timer = 1, .channel = 4, .htim = &htim1},
        {.timer = 4, .channel = 3, .htim = &htim4},
        {.timer = 5, .channel = 1, .htim = &htim5},
        {.timer = 5, .channel = 2, .htim = &htim5},
        {.timer = 5, .channel = 3, .htim = &htim5},
        {.timer = 8, .channel = 1, .htim = &htim8},
        {.timer = 8, .channel = 2, .htim = &htim8},
        {.timer = 8, .channel = 3, .htim = &htim8},
        {.timer = 10, .channel = 1, .htim = &htim10},
    };
    for (auto& context : contexts) {
        if (context.timer == timer && context.channel == timer) {
            context_ = &context;
            break;
        }
    }
}

bool Pwm::set_frequency(const float hz) const {
    if (!context_)
        return false;

    // APB1 定时器时钟为 HCLK / 2，APB2 定时器时钟为 HCLK
    const auto clock = context_->timer == 4 || context_->timer == 5 ? SystemCoreClock / 2 : SystemCoreClock;

    // 计算 PSC、ARR、CCR
    const float period = static_cast<float>(clock) / hz;
    const auto psc = static_cast<uint32_t>(std::ceil(period / 65535.0f) - 1);
    const auto arr = static_cast<uint32_t>(std::round(period / static_cast<float>(psc + 1)) - 1);
    const auto ccr = static_cast<uint32_t>(static_cast<float>(arr + 1) * context_->duty);

    // 设置 PSC、ARR、CCR
    __HAL_TIM_SET_PRESCALER(context_->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(context_->htim, arr);
    for (int channel = 1; channel <= 4; channel++) {
        if (const auto* context = Pwm(context_->timer, channel).context_) {
            __HAL_TIM_SET_COMPARE(context->htim, (channel - 1) * 4, ccr);
        }
    }

    // 触发更新时间，使预分频系数生效
    HAL_TIM_GenerateEvent(context_->htim, TIM_EVENTSOURCE_UPDATE);

    return true;
}

bool Pwm::set_duty(const float duty) const {
    if (!context_)
        return false;

    context_->duty = std::clamp(duty, 0.0f, 1.0f);

    const auto arr = __HAL_TIM_GET_AUTORELOAD(context_->htim);
    const auto ccr = static_cast<uint32_t>(static_cast<float>(arr + 1) * context_->duty);
    __HAL_TIM_SET_COMPARE(context_->htim, (context_->channel - 1) * 4, ccr);

    return true;
}
