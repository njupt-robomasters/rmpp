#include "bsp/storage.hpp"
#include "flashdb/flashdb.h"

struct Storage::Context {
    const char* name_{};
    fdb_kvdb db{};
};

Storage::Storage(const char* name) {
    static constinit Context context{};
    context_ = &context;

    if (!context_->name_) { // 第一次实例化，初始化数据库
        context_->name_ = name;
        fdb_kvdb_init(&context_->db, name, "fdb_kvdb1", nullptr, nullptr);

        // 名称标记：更换名称时格式化数据库
        float marker = 0;
        fdb_blob blob{};
        fdb_blob_make(&blob, &marker, sizeof(marker));

        if (fdb_kv_get_blob(&context_->db, name, &blob) == 0) { // 未找到标记，格式化数据库
            fdb_kv_set_default(&context_->db);
            fdb_kv_set_blob(&context_->db, name, &blob); // 设置标记
        }
    }
}

float Storage::read(const char* key) {
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_get_blob(&context_->db, key, &blob) == sizeof(value) ? value : 0;
}

void Storage::write(const char* key, float value) {
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    fdb_kv_set_blob(&context_->db, key, &blob);
}
