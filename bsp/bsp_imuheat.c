#include "bsp_imuheat.h"
#include "tim.h"

void BSP_IMUHeat_Init() {
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    BSP_IMUHeat_SetPower(0);
}

void BSP_IMUHeat_SetPower(float power) {
    if (power < 0)
        power = 0;
    if (power > 100)
        power = 100;

    const uint16_t ccr = (uint16_t) (power / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim10));
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ccr);
}
