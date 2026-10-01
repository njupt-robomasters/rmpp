#include "misc.hpp"
#include "bsp/timer.hpp"
#include <cmath>

// RGB 色彩循环
static constexpr float RGB_PERIOD = 1; // 循环周期，s
static float hue = 0;
static Timer rgb_timer;

void misc_loop() {
    hue = std::fmod(hue + rgb_timer.restart() / RGB_PERIOD, 1.0f);
    rgb.set_hsv(hue, 1, 1);
}
