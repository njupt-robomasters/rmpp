#pragma once

#include <cstdint>

// 基于 64 位 CPU 周期计数的计时器；不限幅，积分步长由调用方按用途限幅
class Timer {
  private:
    float dt = 0; // 最后一次 restart() 的原始间隔，单位 s，供调试观察
    uint64_t last_cycles = 0;

  public:
    // 延时指定秒数
    static void delay(float seconds);

    // 读取自开始计时以来的秒数
    float elapsed() const;

    // 读取间隔并重新开始计时
    float restart();
};
