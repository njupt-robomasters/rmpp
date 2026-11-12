#include "bsp/monitor.hpp"
#include "referee/referee.hpp"

static Referee referee({.bus = Can(1)});

static void setup() {}

static void loop() {
    referee.update();
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
