// CDC 收发：收到的数据原样发回，另每秒发送一行计数
#include "bsp/cdc.hpp"
#include "bsp/monitor.hpp"
#include <cstdio>

static Cdc cdc1(1);
static Cdc cdc2(2);

static void setup() {
    cdc1.add_rx_callback([](const uint8_t* data, const std::size_t size) { cdc1.send(data, size); });
    cdc2.add_rx_callback([](const uint8_t* data, const std::size_t size) { cdc2.send(data, size); });
}

static void loop() {
    // 1 Hz 打印
    static Timer print_timer;
    if (print_timer.elapsed() < 1)
        return;
    print_timer.restart();

    static int cnt = 0;
    char text[32];
    // CDC1
    const int size1 = std::snprintf(text, sizeof(text), "[cdc1] %d\n", cnt++);
    cdc1.send(reinterpret_cast<const uint8_t*>(text), size1);
    // CDC2
    const int size2 = std::snprintf(text, sizeof(text), "[cdc2] %d\n", cnt++);
    cdc2.send(reinterpret_cast<const uint8_t*>(text), size2);
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
