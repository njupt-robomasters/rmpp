#pragma once

#include "lib/flashdb.h"

class FlashDB {
public:
    FlashDB() = default;

    FlashDB(const char* name);

    // 读取数据库
    float Read(const char* key);

    // 写入数据库
    void Write(const char* key, float value);

private:
    const char* name;
    fdb_kvdb kvdb{};
    bool is_init = false;

    // 初始化数据库
    void init();
};