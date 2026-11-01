#pragma once

#include <functional>
#include <span>

// 板载无源蜂鸣器：直驱发声，或作 PWM-DAC 输出采样流
class Buzzer {
  public:
    // 采样流回调：在 DMA 中断内执行，写入下一块占空比，范围 [0, 1]
    using Fill = std::function<void(std::span<float> duty)>;

    // 以 50% 占空比按 hz 发声，hz 须大于 0
    void set_tone(float hz) const;

    void mute() const;

    // 载波 carrier_hz 即采样率，每个载波周期输出一个占空比；开启前先调用两次 fill 填满缓冲
    // fill 每 1 ms 调用一次，duty 长度为 carrier_hz × 1 ms；carrier_hz 取 1 kHz 的整数倍，不超过 64 kHz
    void start_stream(float carrier_hz, Fill fill) const;

    // 关闭采样流并静音；返回后不再调用 fill
    void stop_stream() const;
};
