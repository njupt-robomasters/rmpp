/*
 * CubeMX 配置：使用片内 Flash，无需启用专用外设
 *
 * 配置与存储布局
 * | 位置                           | 项目            | 设置或要求                                   |
 * | ------------------------------ | --------------- | -------------------------------------------- |
 * | CubeMX                         | DMA / NVIC      | 无需添加 DMA 或中断                          |
 * | FlashDB FAL 分区               | fdb_kvdb1       | 0x080C0000–0x080FFFFF，256 KiB，Sector 10–11 |
 * | 链接布局（非 CubeMX 外设参数） | 固件 Flash 范围 | 不得与上述存储分区重叠                       |
 *
 * 分区由 fal_cfg.h 定义，首次读写时初始化 FlashDB
 */

#include "bsp/storage.hpp"
#include "flashdb/flashdb.h"

struct Storage::Context {
    fdb_kvdb db{};
    bool initialized = false;
};

Storage::Storage(const char* name)
    : name_(name), context_(new Context{}) {}

Storage::~Storage() {
    if (context_->initialized)
        fdb_kvdb_deinit(&context_->db);
    delete context_;
}

bool Storage::init() {
    if (context_->initialized)
        return true;
    if (!name_ || !*name_ || fdb_kvdb_init(&context_->db, name_, "fdb_kvdb1", nullptr, nullptr) != FDB_NO_ERR)
        return false;

    // 保留原有应用标记：更换应用时重置该分区的配置
    float marker = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &marker, sizeof(marker));
    fdb_kv_get_blob(&context_->db, name_, &blob);
    if (blob.saved.len == 0) {
        if (fdb_kv_set_default(&context_->db) != FDB_NO_ERR || fdb_kv_set_blob(&context_->db, name_, &blob) != FDB_NO_ERR) {
            fdb_kvdb_deinit(&context_->db);
            return false;
        }
    }
    context_->initialized = true;
    return true;
}

float Storage::read(const char* key) {
    if (!key || !init())
        return 0;
    float value = 0;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_get_blob(&context_->db, key, &blob) == sizeof(value) ? value : 0;
}

bool Storage::write(const char* key, float value) {
    if (!key || !init())
        return false;
    fdb_blob blob{};
    fdb_blob_make(&blob, &value, sizeof(value));
    return fdb_kv_set_blob(&context_->db, key, &blob) == FDB_NO_ERR;
}
