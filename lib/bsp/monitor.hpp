#pragma once

#include "bsp/timer.hpp"

class Monitor {
  public:
    volatile float freq = 0, freq_lpf = 0;
    volatile float cpu = 0, cpu_lpf = 0;

    void begin() {
        idle_time = timer.elapsed();
    }

    void end() {
        busy_time = timer.restart();

        freq = 1 / (idle_time + busy_time);
        freq_lpf += ALPHA * (freq - freq_lpf);

        cpu = busy_time / (idle_time + busy_time);
        cpu_lpf += ALPHA * (cpu - cpu_lpf);
    }

  private:
    static constexpr float ALPHA = 0.001f;

    Timer timer;
    float idle_time = 0, busy_time = 0;
};
