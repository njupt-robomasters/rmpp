#pragma once

#include "bsp/pwm.hpp"

class Rgb {
  public:
    struct Config {
        Pwm red, green, blue;
    } config;

    explicit Rgb(const Config& config)
        : config(config) {}

    // 各分量范围 [0, 1]
    void set_rgb(float red, float green, float blue) const;
    void set_hsv(float hue, float saturation, float brightness) const;
};
