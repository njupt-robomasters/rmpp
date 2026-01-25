#include "bsp_led.hpp"

#include <cmath>
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

void LED::SetRGB(UnitFloat<> red, UnitFloat<> green, UnitFloat<> blue) {
    red = unit::clamp(red, 0 * ratio, 1 * ratio);
    green = unit::clamp(green, 0 * ratio, 1 * ratio);
    blue = unit::clamp(blue, 0 * ratio, 1 * ratio);

    const auto red_ccr = (uint16_t)((ARR + 1) * red.toFloat(ratio));
    const auto green_ccr = (uint16_t)((ARR + 1) * green.toFloat(ratio));
    const auto blue_ccr = (uint16_t)((ARR + 1) * blue.toFloat(ratio));

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green_ccr);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red_ccr);
}

void LED::SetHSV(UnitFloat<> hue, UnitFloat<> saturation, UnitFloat<> brightness) {
    hue = unit::clamp(hue, 0 * ratio, 1 * ratio);
    saturation = unit::clamp(saturation, 0 * ratio, 1 * ratio);
    brightness = unit::clamp(brightness, 0 * ratio, 1 * ratio);

    // HSV 到 RGB 转换
    const UnitFloat<ratio> c = brightness * saturation;
    const UnitFloat<ratio> x = c * (1.0f - fabsf(fmodf(hue.toFloat() * 6.0f, 2.0f) - 1.0f));
    const UnitFloat<ratio> m = brightness - c;

    UnitFloat<> r, g, b;

    if (hue < 60.0f / 360.0f) {
        r = c;
        g = x;
        b = 0 * ratio;
    } else if (hue < 120.0f / 360.0f) {
        r = x;
        g = c;
        b = 0 * ratio;
    } else if (hue < 180.0f / 360.0f) {
        r = 0 * ratio;
        g = c;
        b = x;
    } else if (hue < 240.0f / 360.0f) {
        r = 0 * ratio;
        g = x;
        b = c;
    } else if (hue < 300.0f / 360.0f) {
        r = x;
        g = 0 * ratio;
        b = c;
    } else {
        r = c;
        g = 0 * ratio;
        b = x;
    }

    SetRGB(r + m, g + m, b + m);
}