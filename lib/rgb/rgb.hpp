#pragma once

#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"

class Rgb {
  public:
    struct Config {
        Pwm red, green, blue;
        float period = 1; // 颜色循环周期，秒
        float saturation = 1;
        float brightness = 1;
    } config;

    explicit Rgb(const Config& config) : config(config) {}

    explicit operator bool() const {
        return config.red && config.green && config.blue;
    }

    bool set_rgb(float red, float green, float blue) const;
    bool set_hsv(float hue, float saturation, float brightness) const;
    void on_loop();

  private:
    float hue_ = 0;
    Timer timer_;
};
