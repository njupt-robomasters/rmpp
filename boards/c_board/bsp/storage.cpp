#include "bsp/storage.hpp"
#include "flashdb/flashdb.h"

static const char* db_name = nullptr;
static fdb_kvdb db{};

Storage::Storage(const char* name) {
    if (db_name) // 已由先构造的实例初始化
        return;
    db_name = name;
    fdb_kvdb_init(&db, name, "fdb_kvdb1", nullptr, nullptr);

    // 名称标记：更换名称时格式化数据库
    float marker = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &marker, sizeof(marker));
    if (fdb_kv_get_blob(&db, name, &blob) == 0) {
        fdb_kv_set_default(&db);
        fdb_kv_set_blob(&db, name, &blob);
    }
}

float Storage::read(const char* key) {
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_get_blob(&db, key, &blob) == sizeof(value) ? value : 0;
}

void Storage::write(const char* key, float value) {
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_set_blob(&db, key, &blob);
}
