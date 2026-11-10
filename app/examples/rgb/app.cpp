#include "bsp/monitor.hpp"
#include "bsp/rgb.hpp"

static Rgb rgb;

static void setup() {}

static void loop() {
    // 色相循环
    static float hue = 0;
    hue = std::fmod(hue + 0.001f, 1.0f);
    rgb.set_hsv(hue, 1, 1);
}

extern "C" void app_main() {
    setup();

    while (true) {
        // 1 kHz 更新
        static Timer app_timer;
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        // 负载监控
        static Monitor app_monitor;
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
