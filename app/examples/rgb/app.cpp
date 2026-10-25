#include "bsp/timer.hpp"
#include "misc/rgb.hpp"

static Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});

extern "C" void app_main() {
    static constexpr float PERIOD = 1; // 1 s 周期色彩循环
    float hue = 0;

    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            hue = std::fmod(hue + timer.restart() / PERIOD, 1.0f);
            rgb.set_hsv(hue, 1, 1);
        }
    }
}
