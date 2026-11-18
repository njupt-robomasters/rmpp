// KvStore 示例：每次上电 boot_count 加 1 并写回
#include "bsp/kv_store.hpp"
#include "bsp/monitor.hpp"

// 构造不访问 Flash，首次读写时初始化数据库
static KvStore kv_store("kv_store_example");

static void setup() {
    const float boot_count = kv_store.read("boot_count") + 1;
    kv_store.write("boot_count", boot_count);
}

static void loop() {}

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
