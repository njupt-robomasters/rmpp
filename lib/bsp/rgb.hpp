#pragma once

#include <cmath>

// 板载三色 LED：C 板为 TIM5 PWM，mc02 为 WS2812
class Rgb {
  public:
    // 各分量范围 [0, 1]
    void set(float red, float green, float blue) const;

    // 色相、饱和度、明度范围 [0, 1]
    void set_hsv(const float hue, const float saturation, const float value) const {
        const float c = value * saturation;
        const float x = c * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
        const float m = value - c;
        if (hue < 1.0f / 6)
            set(c + m, x + m, m);
        else if (hue < 2.0f / 6)
            set(x + m, c + m, m);
        else if (hue < 3.0f / 6)
            set(m, c + m, x + m);
        else if (hue < 4.0f / 6)
            set(m, x + m, c + m);
        else if (hue < 5.0f / 6)
            set(x + m, m, c + m);
        else
            set(c + m, m, x + m);
    }
};
