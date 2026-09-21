#pragma once

#include "lib/flashdb.h"

class FlashDb {
  public:
    FlashDb() = default;

    FlashDb(const char* name);

    // 读取数据库
    float read(const char* key);

    // 写入数据库
    void write(const char* key, float value);

  private:
    const char* name_;
    fdb_kvdb kvdb_{};
    bool is_init_ = false;

    // 初始化数据库
    void init();
};