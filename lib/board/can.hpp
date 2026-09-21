#pragma once

#include <array>
#include <cstdint>
#include <functional>

namespace rmpp {
struct CanFrame {
    std::uint32_t id = 0;
    std::array<std::uint8_t, 8> data{};
    std::uint8_t size = 0;
    bool extended = false;
    bool remote = false;
};

class Can {
  public:
    using RxCallback = std::function<void(const CanFrame&)>;
    Can() = default;
    explicit operator bool() const { return state_ != nullptr; }
    bool send(const CanFrame& frame) const;
    bool add_rx_callback(RxCallback callback) const;
    float cpu_usage() const;
    std::uint32_t bus_off_count() const;

  private:
    struct State;
    State* state_ = nullptr;
    explicit Can(State* state) : state_(state) {}
    static State* find(int id);
    friend class Board;
    friend void can_init();
    friend void can_receive(int id, std::uint32_t fifo);
    friend void can_error(int id);
};
} // namespace rmpp
