#pragma once

#include "bsp/timer.hpp"
#include "math/filter.hpp"

// 统计周期任务的频率与占用：begin 标记任务开始，end 标记任务结束
class Monitor {
  public:
    volatile float freq = 0, freq_lpf = 0; // Hz
    volatile float cpu = 0, cpu_lpf = 0;   // 任务耗时占周期比例，[0, 1]

    void begin() {
        idle_time_ = timer_.elapsed();
    }

    void end() {
        const float busy_time = timer_.restart();
        const float period = idle_time_ + busy_time;

        freq = 1 / period;
        freq_lpf = lowpass<float>(freq_lpf, freq, CUTOFF_HZ, period);

        cpu = busy_time / period;
        cpu_lpf = lowpass<float>(cpu_lpf, cpu, CUTOFF_HZ, period);
    }

  private:
    static constexpr float CUTOFF_HZ = 0.2f; // 低通截止频率，时间常数约 0.8 s

    Timer timer_;
    float idle_time_ = 0;
};
