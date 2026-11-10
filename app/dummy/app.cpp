#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "referee.hpp"

static void setup() {
    melody.play(C5D5G5);
}

static void loop() {
    melody.update();
    fsi6x.update();

    referee_loop();
    chassis_loop();
    gimbal_loop();
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
