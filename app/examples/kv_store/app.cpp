// KvStore 示例：每次上电 boot_count 加 1 并写回，重新上电后用调试器查看是否递增
#include "bsp/kv_store.hpp"

static float boot_count = 0;      // 本次上电写入的计数
static bool is_read_back = false; // 写后读回是否一致

extern "C" void app_main() {
    // 须在 bsp_init 之后构造，首次构造时初始化 Flash 与数据库
    KvStore kv_store("kv_store_example");

    boot_count = kv_store.read("boot_count") + 1;
    kv_store.write("boot_count", boot_count);
    is_read_back = kv_store.read("boot_count") == boot_count;

    while (true) {
    }
}
