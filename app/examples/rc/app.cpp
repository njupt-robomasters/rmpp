#include "bsp/monitor.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

static void setup() {}

static void loop() {
    vt13.update();
    fsi6x.update();
}

static Timer app_timer;     // 主循环 1 kHz 分频
static Monitor app_monitor; // 主循环负载监控

extern "C" void app_main() {
    setup();
    while (true) {
        // 1 kHz 更新
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        // 负载监控
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
