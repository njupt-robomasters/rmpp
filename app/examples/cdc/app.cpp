// CDC 收发：收到的数据原样发回，另每秒发送一行计数
#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include <cstdio>

static constexpr Cdc cdc(1);

extern "C" void app_main() {
    cdc.add_rx_callback([](const uint8_t* data, const std::size_t size) { cdc.send(data, size); });

    int count = 0;
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1) {
            timer.restart();
            char text[32];
            const int size = std::snprintf(text, sizeof(text), "tick %d\r\n", count++);
            cdc.send(reinterpret_cast<const uint8_t*>(text), size);
        }
    }
}
