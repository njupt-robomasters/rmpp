#pragma once

#include "bsp/bytes.hpp"
#include <algorithm>

void timer_init();
void pwm_init();
void can_init();
void can_receive(int id, std::uint32_t fifo);
void can_error(int id);
void uart_init();
void uart_receive(int id, std::uint16_t size);
void uart_error(int id);
void cdc_receive(Bytes bytes);

// 按实际时间累计回调负载；没有新回调时保留上次结果。
struct CallbackTiming {
    std::uint32_t window_start_ms = 0;
    float busy_us = 0;

    void record(float duration_us, std::uint32_t now_ms, volatile float& usage) {
        busy_us += duration_us;
        const auto elapsed_ms = now_ms - window_start_ms;
        if (elapsed_ms < 100)
            return;

        const float sample = std::clamp(busy_us / (elapsed_ms * 1000.0f), 0.0f, 1.0f);
        const float alpha = elapsed_ms / (400.0f + elapsed_ms);
        const float previous = usage;
        usage = previous + alpha * (sample - previous);
        busy_us = 0;
        window_start_ms = now_ms;
    }
};
