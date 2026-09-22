#pragma once

#include "bytes.hpp"
#include <functional>

class Cdc {
  public:
    using RxCallback = std::function<void(Bytes)>;
    Cdc() = default;
    explicit Cdc(int id);

    explicit operator bool() const {
        return state_ != nullptr;
    }

    bool send(Bytes bytes) const;
    bool printf(const char* format, ...) const;
    bool add_rx_callback(RxCallback callback) const;

  private:
    struct State;
    int id_ = 0;
    State* state_ = nullptr;
    friend void cdc_receive(Bytes bytes);
};
