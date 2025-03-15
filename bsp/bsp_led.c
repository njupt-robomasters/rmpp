#include "bsp_led.h"
#include "tim.h"

#include <stdint.h>

const float RGBA_MAX = 255;

void BSP_LED_Init() {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void BSP_LED_ShowRGBA(const float red, const float green, const float blue, const float alpha) {
    const uint16_t pwm_max = __HAL_TIM_GetAutoreload(&htim5);

    const float alpha_norm = alpha / RGBA_MAX;
    const uint16_t red_pwm = (uint16_t) (red * alpha_norm / RGBA_MAX * (float) pwm_max);
    const uint16_t green_pwm = (uint16_t) (green * alpha_norm / RGBA_MAX * (float) pwm_max);
    const uint16_t blue_pwm = (uint16_t) (blue * alpha_norm / RGBA_MAX * (float) pwm_max);

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue_pwm);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green_pwm);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red_pwm);
}
