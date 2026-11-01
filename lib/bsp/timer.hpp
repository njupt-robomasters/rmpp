#pragma once

#include <cmath>
#include <cstdint>

// 基于 64 位 CPU 周期计数的计时器
class Timer {
  public:
    float dt = 0; // 最近一次读取的间隔，s，已限幅到 [min_dt, max_dt]

    constexpr Timer(const float min_dt = 0, const float max_dt = INFINITY) : min_dt_(min_dt), max_dt_(max_dt) {}

    // 延时指定秒数
    static void delay(float seconds);

    // 读取自开始计时以来的秒数
    float elapsed();

    // 读取间隔并重新开始计时
    float restart();

  private:
    const float min_dt_, max_dt_;
    uint64_t last_cycles_ = 0;
};
