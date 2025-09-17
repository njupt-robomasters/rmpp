#include "bsp_imuheat.hpp"
#include "tim.h"

using namespace BSP;

void IMUHeat::Init() {
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    SetPower(0);
}

void IMUHeat::SetPower(float power) {
    if (power < 0) power = 0;
    if (power > 1) power = 1;

    const uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim10);
    const auto ccr = (uint16_t)(power * (float)arr);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ccr);
}
