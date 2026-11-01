#pragma once

// 持久化键值参数，由板卡以 FlashDB 实现
class KvStore {
  public:
    // 所有实例共享同一数据库，首次构造时初始化；name 为应用标记，更换后会格式化数据库，字符串须保持有效
    explicit KvStore(const char* name);

    // 缺失键读得 0
    float read(const char* key);
    void write(const char* key, float value);
};
