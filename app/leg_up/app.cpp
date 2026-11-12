#include "app.hpp"
#include "bsp/monitor.hpp"
#include "melody/music/c5d5g5.hpp"
#include "shooter.hpp"
#include "gimbal.hpp"

static void setup() {
    melody.play(C5D5G5);
}

static void loop() {
    melody.update();
    vt13.update();

    shooter_loop();
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
