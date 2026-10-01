#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "misc/rgb.hpp"
#include <cmath>

static Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});

extern "C" void app_main() {
    // 1 s 周期色彩循环
    static constexpr float PERIOD = 1;
    float hue = 0;

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            hue = std::fmod(hue + timer.restart() / PERIOD, 1.0f);
            rgb.set_hsv(hue, 1, 1);
        }
    }
}
