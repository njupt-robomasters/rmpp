#include "rgb.hpp"
#include <algorithm>
#include <cmath>

bool Rgb::set_rgb(float red, float green, float blue) const {
    if (!*this || !std::isfinite(red) || !std::isfinite(green) || !std::isfinite(blue))
        return false;
    const bool r = config.red.set_duty(red);
    const bool g = config.green.set_duty(green);
    const bool b = config.blue.set_duty(blue);
    return r && g && b;
}

bool Rgb::set_hsv(float hue, float saturation, float brightness) const {
    if (!*this || !std::isfinite(hue) || !std::isfinite(saturation) || !std::isfinite(brightness))
        return false;
    hue = std::clamp(hue, 0.0f, 1.0f);
    saturation = std::clamp(saturation, 0.0f, 1.0f);
    brightness = std::clamp(brightness, 0.0f, 1.0f);
    const float c = brightness * saturation;
    const float x = c * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
    const float m = brightness - c;
    if (hue < 1.0f / 6)
        return set_rgb(c + m, x + m, m);
    if (hue < 2.0f / 6)
        return set_rgb(x + m, c + m, m);
    if (hue < 3.0f / 6)
        return set_rgb(m, c + m, x + m);
    if (hue < 4.0f / 6)
        return set_rgb(m, x + m, c + m);
    if (hue < 5.0f / 6)
        return set_rgb(x + m, m, c + m);
    return set_rgb(c + m, m, x + m);
}

void Rgb::on_loop() {
    const float dt = timer_.update_dt();
    if (!std::isfinite(config.period) || config.period <= 0)
        return;
    hue_ = std::fmod(hue_ + dt / config.period, 1.0f);
    set_hsv(hue_, config.saturation, config.brightness);
}
