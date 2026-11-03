#include "bsp/kv_store.hpp"
#include "flashdb.h"

static const char* db_name = nullptr;
static bool is_db_ready = false;
static fdb_kvdb db{};

// 首次读写时初始化数据库，此时 bsp_init 须已完成
static void prepare_db() {
    if (is_db_ready)
        return;
    is_db_ready = true;
    fdb_kvdb_init(&db, db_name, "fdb_kvdb1", nullptr, nullptr);

    // 名称标记：更换名称时格式化数据库
    float marker = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &marker, sizeof(marker));
    if (fdb_kv_get_blob(&db, db_name, &blob) == 0) {
        fdb_kv_set_default(&db);
        fdb_kv_set_blob(&db, db_name, &blob);
    }
}

KvStore::KvStore(const char* name) {
    if (!db_name) // 只记录首个实例的名称，不访问 Flash
        db_name = name;
}

float KvStore::read(const char* key) {
    prepare_db();

    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_get_blob(&db, key, &blob) == sizeof(value) ? value : 0;
}

void KvStore::write(const char* key, float value) {
    prepare_db();

    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_set_blob(&db, key, &blob);
}
