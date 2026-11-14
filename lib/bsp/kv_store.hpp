#pragma once

// 持久化键值参数，由板卡以 FlashDB 实现
class KvStore {
  public:
    // 所有实例共享同一数据库，取首个构造实例的 name 为应用标记，更换后会格式化数据库，字符串须保持有效
    // 构造不访问 Flash，可早于 bsp_init（如全局对象）
    explicit KvStore(const char* name);

    // 首次读写时初始化数据库，须在 bsp_init 之后调用；缺失键读得 0
    float read(const char* key);

    void write(const char* key, float value);
};
