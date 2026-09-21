#pragma once

#include "bytes.hpp"
#include <functional>

namespace rmpp {
class Uart {
  public:
    using RxCallback = std::function<void(Bytes)>;
    Uart() = default;
    explicit operator bool() const { return state_ != nullptr; }
    bool send(Bytes bytes) const;
    bool add_rx_callback(RxCallback callback) const;
    float cpu_usage() const;
    std::uint32_t error_count() const;

  private:
    struct State;
    State* state_ = nullptr;
    explicit Uart(State* state) : state_(state) {}
    static State* find(int id);
    friend class Board;
    friend void uart_init();
    friend void uart_receive(int id, std::uint16_t size);
    friend void uart_error(int id);
};
} // namespace rmpp
