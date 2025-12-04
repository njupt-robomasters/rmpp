#include "bsp_imuheat.hpp"

#include <algorithm>

#include "tim.h"

using namespace BSP;

static constexpr uint16_t PSC = 168-1;
static constexpr uint16_t ARR = 1000-1;

void IMUHeat::Init() {
    __HAL_TIM_SET_PRESCALER(&htim10, PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim10, ARR);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    SetPower(0);
}

void IMUHeat::SetPower(float power) {
    power = std::clamp(power, 0.0f, 1.0f);

    const uint16_t ccr = (uint16_t)((ARR + 1) * power);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ccr);
}
