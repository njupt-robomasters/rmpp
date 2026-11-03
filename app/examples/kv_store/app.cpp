// KvStore 示例：每次上电 boot_count 加 1 并写回
#include "bsp/kv_store.hpp"

// 构造不访问 Flash，首次读写时初始化数据库
static KvStore kv_store("kv_store_example");

extern "C" void app_main() {
    const float boot_count = kv_store.read("boot_count") + 1;
    kv_store.write("boot_count", boot_count);

    while (true) {
    }
}
