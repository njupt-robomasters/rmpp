#include "bsp/rgb.hpp"
#include "bsp/timer.hpp"

static Rgb rgb;

extern "C" void app_main() {
    while (true) {
        // 1 kHz 更新
        static Timer timer;
        if (timer.elapsed() < 0.001f)
            continue;
        timer.restart();

        // 色相循环
        static float hue = 0;
        hue = std::fmod(hue + 0.001f, 1.0f);
        rgb.set_hsv(hue, 1, 1);
    }
}
