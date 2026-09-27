#pragma once

class Storage {
  public:
    explicit Storage(const char* name);
    ~Storage();
    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;

    float read(const char* key);
    bool write(const char* key, float value);

  private:
    struct Context;
    const char* name_;
    Context* context_;
    bool init();
};
