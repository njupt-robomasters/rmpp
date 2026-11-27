#pragma once

#include <cstdint>
#include <functional>

struct CanFrame {
    uint32_t id = 0;
    bool is_extend = false;
    bool is_remote = false;
    uint8_t size = 0;
    uint8_t data[8]{};
};

class Can {
  public:
    using RxCallback = std::function<void(const CanFrame&)>;

  public:
    constexpr explicit Can(int) {}

    bool can_send() const { return true; }

    void send(const CanFrame&) const {}

    void add_rx_callback(RxCallback) const {}
};
