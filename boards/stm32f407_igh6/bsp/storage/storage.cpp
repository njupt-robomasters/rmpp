#include "bsp/storage.hpp"
#include "flashdb/flashdb.h"

struct Storage::State {
    fdb_kvdb db{};
    bool initialized = false;
};

Storage::Storage(const char* name) : name_(name), state_(new State{}) {}

Storage::~Storage() {
    if (state_->initialized)
        fdb_kvdb_deinit(&state_->db);
    delete state_;
}

bool Storage::init() {
    if (state_->initialized)
        return true;
    if (!name_ || !*name_ || fdb_kvdb_init(&state_->db, name_, "fdb_kvdb1", nullptr, nullptr) != FDB_NO_ERR)
        return false;

    // 保留原有应用标记：更换应用时重置该分区的配置。
    float marker = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &marker, sizeof(marker));
    fdb_kv_get_blob(&state_->db, name_, &blob);
    if (blob.saved.len == 0) {
        if (fdb_kv_set_default(&state_->db) != FDB_NO_ERR || fdb_kv_set_blob(&state_->db, name_, &blob) != FDB_NO_ERR) {
            fdb_kvdb_deinit(&state_->db);
            return false;
        }
    }
    state_->initialized = true;
    return true;
}

float Storage::read(const char* key) {
    if (!key || !init())
        return 0;
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_get_blob(&state_->db, key, &blob) == sizeof(value) ? value : 0;
}

bool Storage::write(const char* key, float value) {
    if (!key || !init())
        return false;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_set_blob(&state_->db, key, &blob) == FDB_NO_ERR;
}
