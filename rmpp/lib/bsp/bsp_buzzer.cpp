#include "bsp_buzzer.hpp"
#include "tim.h"

using namespace BSP;

static constexpr uint16_t PSC = 168 - 1;

void BUZZER::Init() {
    __HAL_TIM_SET_PRESCALER(&htim4, PSC);
}

void BUZZER::Play(const UnitFloat<>& freq) {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    const uint16_t arr = (uint16_t)((float)SystemCoreClock / (float)(PSC + 1) / freq.toFloat(Hz)) - 1;
    const uint16_t ccr = (arr + 1) / 2 - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void BUZZER::Stop() {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}