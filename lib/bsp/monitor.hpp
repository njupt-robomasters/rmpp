#pragma once

#include "bsp/timer.hpp"
#include "math/filter.hpp"

// 统计周期任务的频率与占用：begin 标记任务开始，end 标记任务结束
class Monitor {
  private:
    static constexpr float CUTOFF_HZ = 0.2f; // 低通截止频率，时间常数约 0.8 s

    volatile float freq = 0, freq_lpf = 0; // Hz
    volatile float cpu = 0, cpu_lpf = 0;   // 任务耗时占周期比例，[0, 1]
    float idle_time = 0;
    Timer period_timer;

  public:
    void begin() {
        idle_time = period_timer.elapsed();
    }

    void end() {
        // 周期为相邻两次 end 的间隔，begin 前为空闲
        const float period = period_timer.restart();
        const float busy_time = period - idle_time;

        freq = 1 / period;
        freq_lpf = lowpass<float>(freq_lpf, freq, CUTOFF_HZ, period);

        cpu = busy_time / period;
        cpu_lpf = lowpass<float>(cpu_lpf, cpu, CUTOFF_HZ, period);
    }
};
