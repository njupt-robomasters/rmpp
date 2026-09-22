#include "bsp/storage.hpp"
extern "C" {
#include "flashdb.h"
}
// FAL 的断言宏可能覆盖标准 assert，测试重新启用标准断言。
#include <cassert>
#include <cmath>
#include <cstdio>

extern unsigned flash_reads;
extern unsigned flash_writes;
extern unsigned flash_erases;

static Storage global_storage("global");

int main() {
    assert(flash_reads == 0 && flash_writes == 0 && flash_erases == 0);
    {
        Storage storage("hero");
        assert(storage.read("missing") == 0);
        assert(storage.write("yaw_offset", 1.25f));
        const auto writes = flash_writes;
        assert(storage.read("yaw_offset") == 1.25f);
        assert(storage.read("yaw_offset") == 1.25f && flash_writes == writes);
    }
    {
        Storage storage("hero");
        assert(storage.read("yaw_offset") == 1.25f);
        assert(storage.write("yaw_offset", -2.5f));
    }
    // 用原 FlashDB API 读取同一 blob，验证格式与应用标记兼容。
    {
        fdb_kvdb db{};
        assert(fdb_kvdb_init(&db, "hero", "fdb_kvdb1", nullptr, nullptr) == FDB_NO_ERR);
        float value = 0;
        fdb_blob blob{};
        fdb_blob_make(&blob, &value, sizeof(value));
        assert(fdb_kv_get_blob(&db, "yaw_offset", &blob) == sizeof(value) && value == -2.5f);
        value = 3.75f;
        assert(fdb_kv_set_blob(&db, "yaw_offset", &blob) == FDB_NO_ERR);
        fdb_kvdb_deinit(&db);
    }
    {
        Storage storage("hero");
        assert(storage.read("yaw_offset") == 3.75f);
    }
    {
        Storage storage("infantry");
        assert(storage.read("yaw_offset") == 0);
        assert(storage.write("yaw_offset", 4));
    }
    {
        Storage storage("hero");
        assert(storage.read("yaw_offset") == 0);
        assert(!storage.write(nullptr, 1));
    }
    Storage invalid(nullptr);
    assert(invalid.read("key") == 0 && !invalid.write("key", 1));
    std::puts("storage: all checks passed");
}
