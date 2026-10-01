#include "rgb.hpp"
#include <cmath>

void Rgb::set_rgb(const float red, const float green, const float blue) const {
    config.red.set_duty(red);
    config.green.set_duty(green);
    config.blue.set_duty(blue);
}

void Rgb::set_hsv(const float hue, const float saturation, const float brightness) const {
    const float c = brightness * saturation;
    const float x = c * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
    const float m = brightness - c;
    if (hue < 1.0f / 6)
        set_rgb(c + m, x + m, m);
    else if (hue < 2.0f / 6)
        set_rgb(x + m, c + m, m);
    else if (hue < 3.0f / 6)
        set_rgb(m, c + m, x + m);
    else if (hue < 4.0f / 6)
        set_rgb(m, x + m, c + m);
    else if (hue < 5.0f / 6)
        set_rgb(x + m, m, c + m);
    else
        set_rgb(c + m, m, x + m);
}
