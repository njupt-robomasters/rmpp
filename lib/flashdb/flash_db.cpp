#include "flash_db.hpp"

FlashDb::FlashDb(const char* name) : name_(name) {}

float FlashDb::read(const char* key) {
    init();

    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_get_blob(&kvdb_, key, &blob);

    return value;
}

void FlashDb::write(const char* key, const float value) {
    init();

    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_set_blob(&kvdb_, key, &blob);
}

void FlashDb::init() {
    if (is_init_)
        return;

    fdb_kvdb_init(&kvdb_, name_, "fdb_kvdb1", nullptr, nullptr);

    // 读取数据库
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_get_blob(&kvdb_, name_, &blob);

    // 读不到数据则格式化数据库
    if (blob.saved.len == 0) {
        fdb_kv_set_default(&kvdb_);
        fdb_kv_set_blob(&kvdb_, name_, &blob);
    }
}