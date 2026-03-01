#include "FlashDB.hpp"

FlashDB::FlashDB(const char* name) : name(name) {}

float FlashDB::Read(const char* key) {
    init();

    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_get_blob(&kvdb, key, &blob);

    return value;
}

void FlashDB::Write(const char* key, const float value) {
    init();

    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_set_blob(&kvdb, key, &blob);
}

void FlashDB::init() {
    if (is_init) return;

    fdb_kvdb_init(&kvdb, name, "fdb_kvdb1", nullptr, nullptr);

    // 读取数据库
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_get_blob(&kvdb, name, &blob);

    // 读不到数据则格式化数据库
    if (blob.saved.len == 0) {
        fdb_kv_set_default(&kvdb);
        fdb_kv_set_blob(&kvdb, name, &blob);
    }
}