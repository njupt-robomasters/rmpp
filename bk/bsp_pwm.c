#include "bsp_pwm.h"
#include "tim.h"

void BSP_PWM_Init() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    BSP_PWM_SetDuty(0);

    // 用于IMU加热
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    BSP_PWM_IMUHeat_SetPower(0);
}

void BSP_PWM_SetDuty(const float duty) {
    const uint16_t tim1_ccr = (uint16_t) (duty / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tim1_ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tim1_ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tim1_ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, tim1_ccr);

    const uint16_t tim8_ccr = (uint16_t) (duty / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim8));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, tim8_ccr);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, tim8_ccr);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, tim8_ccr);
}

void BSP_PWM_IMUHeat_SetPower(float power) {
    if (power < 0)
        power = 0;
    if (power > 100)
        power = 100;

    const uint16_t ccr = (uint16_t) (power / 100.0f * __HAL_TIM_GET_AUTORELOAD(&htim10));
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ccr);
}
