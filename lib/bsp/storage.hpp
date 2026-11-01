#pragma once

class Storage {
  public:
    // 所有实例共享同一数据库，首次构造的 name 决定应用标记
    explicit Storage(const char* name);

    float read(const char* key);
    void write(const char* key, float value);
};
