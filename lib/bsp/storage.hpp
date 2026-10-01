#pragma once

class Storage {
  public:
    // 所有句柄共享唯一 Context，首次成功初始化的 name 决定应用标记
    explicit Storage(const char* name);

    float read(const char* key);
    void write(const char* key, float value);

  private:
    struct Context;
    Context* context_;
};
