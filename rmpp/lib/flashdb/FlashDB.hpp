#pragma once

#include "lib/flashdb.h"

class FlashDB {
public:
    FlashDB() = default;

    // 初始化数据库
    void Init(const char* name);

    // 读取数据库
    float Read(const char* key);

    // 写入数据库
    void Write(const char* key, float value);

private:
    fdb_kvdb kvdb{};
    bool is_init = false;
};