#pragma once

#include "bytes.hpp"
#include <functional>

class Uart {
  public:
    using RxCallback = std::function<void(Bytes)>;
    Uart() = default;
    explicit Uart(int id);

    explicit operator bool() const {
        return state_ != nullptr;
    }

    bool send(Bytes bytes) const;
    bool add_rx_callback(RxCallback callback) const;

  private:
    struct State;
    int id_ = 0;
    State* state_ = nullptr;
    friend void uart_init();
    friend void uart_receive(int id, std::uint16_t size);
    friend void uart_error(int id);
};
