#include "bsp_led.hpp"

#include <cmath>
#include <algorithm>
#include "tim.h"

using namespace BSP;

void LED::Init() {
    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, 65535);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void LED::SetRGB(float red, float green, float blue) {
    red = std::clamp(red, 0.0f, 1.0f);
    green = std::clamp(green, 0.0f, 1.0f);
    blue = std::clamp(blue, 0.0f, 1.0f);

    const auto red_ccr = (uint16_t)(red * 65535.0f);
    const auto green_ccr = (uint16_t)(green * 65535.0f);
    const auto blue_ccr = (uint16_t)(blue * 65535.0f);

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red_ccr);
}

/**
 * @brief 设置 LED 的 HSV 颜色
 * @param hue 色相 (0-360)
 * @param saturation 饱和度 (0-1)
 * @param brightness 亮度 (0-1)
 */
void LED::SetHSV(float hue, float saturation, float brightness) {
    // 确保输入在有效范围内
    hue = fmodf(hue, 360.0f); // 色相循环 0-360
    if (hue < 0) hue += 360.0f;
    saturation = std::clamp(saturation, 0.0f, 1.0f);
    brightness = std::clamp(brightness, 0.0f, 1.0f);

    // HSV 到 RGB 转换
    float c = brightness * saturation;
    float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
    float m = brightness - c;

    float r, g, b;

    if (hue < 60.0f) {
        r = c;
        g = x;
        b = 0;
    } else if (hue < 120.0f) {
        r = x;
        g = c;
        b = 0;
    } else if (hue < 180.0f) {
        r = 0;
        g = c;
        b = x;
    } else if (hue < 240.0f) {
        r = 0;
        g = x;
        b = c;
    } else if (hue < 300.0f) {
        r = x;
        g = 0;
        b = c;
    } else {
        r = c;
        g = 0;
        b = x;
    }

    SetRGB(r + m, g + m, b + m);
}
