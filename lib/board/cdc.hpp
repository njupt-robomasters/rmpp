#pragma once

#include "bytes.hpp"
#include <functional>

namespace rmpp {
class Cdc {
  public:
    using RxCallback = std::function<void(Bytes)>;
    Cdc() = default;
    explicit operator bool() const { return state_ != nullptr; }
    bool send(Bytes bytes) const;
    bool printf(const char* format, ...) const;
    bool add_rx_callback(RxCallback callback) const;
    float cpu_usage() const;

  private:
    struct State;
    State* state_ = nullptr;
    explicit Cdc(State* state) : state_(state) {}
    static State* find(int id);
    friend class Board;
    friend void cdc_receive(Bytes bytes);
};
} // namespace rmpp
