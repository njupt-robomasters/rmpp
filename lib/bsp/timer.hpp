#pragma once

#include <cmath>
#include <cstdint>

class Timer {
  public:
    float dt = 0;

    constexpr Timer(const float min_dt = 0, const float max_dt = INFINITY) : min_dt_(min_dt), max_dt_(max_dt) {}

    // 延时指定秒数
    static void delay(float seconds);

    // 读取自开始计时以来的秒数
    float elapsed();

    // 读取间隔并重新开始计时
    float restart();

    bool operator==(const Timer& other) const {
        return last_ms_ == other.last_ms_ && last_tick_ == other.last_tick_;
    }

  private:
    const float min_dt_, max_dt_;
    uint32_t last_tick_ = 0;
    uint32_t last_ms_ = 0;
};
