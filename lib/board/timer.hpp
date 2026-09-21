#pragma once

#include <cstdint>

namespace rmpp {
class Timer {
  public:
    float dt = 0;
    float freq = 0;
    constexpr explicit Timer(float min_dt = 0.0001f, float max_dt = 10.0f) : min_dt_(min_dt), max_dt_(max_dt) {}
    float get_dt();
    float update_dt();
    bool poll_timeout(float seconds);
    void reset();
    static void delay(float seconds);
    bool operator==(const Timer& other) const { return has_baseline_ == other.has_baseline_ && last_tick_ == other.last_tick_; }

  private:
    float min_dt_, max_dt_;
    std::uint32_t last_tick_ = 0;
    bool has_baseline_ = false;
    bool at_limit_ = false;
};
} // namespace rmpp
