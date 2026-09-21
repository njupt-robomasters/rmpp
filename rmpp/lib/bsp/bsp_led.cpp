#include "bsp_led.hpp"

#include <cmath>
#include "math/Math.hpp"
#include "tim.h"

using namespace BSP;

static constexpr uint16_t PSC = 0;
static constexpr uint16_t ARR = 65535 - 1;

void LED::Init() {
    __HAL_TIM_SET_PRESCALER(&htim5, PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim5, ARR);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void LED::SetRGB(float red, float green, float blue) {
    red = math::clamp(red, 0.0f, 1.0f);
    green = math::clamp(green, 0.0f, 1.0f);
    blue = math::clamp(blue, 0.0f, 1.0f);

    const auto red_ccr = (uint16_t)((ARR + 1) * red);
    const auto green_ccr = (uint16_t)((ARR + 1) * green);
    const auto blue_ccr = (uint16_t)((ARR + 1) * blue);

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red_ccr);
}

void LED::SetHSV(float hue, float saturation, float brightness) {
    hue = math::clamp(hue, 0.0f, 1.0f);
    saturation = math::clamp(saturation, 0.0f, 1.0f);
    brightness = math::clamp(brightness, 0.0f, 1.0f);

    // HSV 到 RGB 转换
    const float c = brightness * saturation;
    const float x = c * (1.0f - std::fabsf(std::fmodf(hue * 6.0f, 2.0f) - 1.0f));
    const float m = brightness - c;

    float r{}, g{}, b{};

    if (hue < 60.0f / 360.0f) {
        r = c;
        g = x;
        b = 0.0f;
    } else if (hue < 120.0f / 360.0f) {
        r = x;
        g = c;
        b = 0.0f;
    } else if (hue < 180.0f / 360.0f) {
        r = 0.0f;
        g = c;
        b = x;
    } else if (hue < 240.0f / 360.0f) {
        r = 0.0f;
        g = x;
        b = c;
    } else if (hue < 300.0f / 360.0f) {
        r = x;
        g = 0.0f;
        b = c;
    } else {
        r = c;
        g = 0.0f;
        b = x;
    }

    SetRGB(r + m, g + m, b + m);
}