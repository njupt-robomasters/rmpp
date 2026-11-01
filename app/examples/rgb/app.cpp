#include "bsp/rgb.hpp"
#include "bsp/timer.hpp"

static Rgb rgb;

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
