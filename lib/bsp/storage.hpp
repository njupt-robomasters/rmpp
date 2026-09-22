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
    struct State;
    const char* name_;
    State* state_;
    bool init();
};
