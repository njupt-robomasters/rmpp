// 主机仿真：用 RC 模型生成 0x051 应答，验证 SuperCap 在线容量/ESR 辨识
// 构建与运行见同目录 README.md
#include "power/super_cap.hpp"
#include <cmath>
#include <cstdio>
#include <random>

// 桥接：仿真时间驱动 Timer，Can 只保存回调
static double sim_time = 0;
static Can::RxCallback rx_callback;

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame&) const {}

void Can::add_rx_callback(RxCallback callback) const {
    rx_callback = callback;
}

void Timer::delay(float) {}

float Timer::elapsed() const {
    return static_cast<float>(sim_time - last_cycles * 1e-6);
}

float Timer::restart() {
    const float dt = static_cast<float>(sim_time - last_cycles * 1e-6);
    last_cycles = static_cast<uint64_t>(sim_time * 1e6);
    return dt;
}

static void put(uint8_t* data, const float value) {
    const auto raw = static_cast<int16_t>(std::lround(value * 1000));
    data[0] = static_cast<uint8_t>(raw >> 8);
    data[1] = static_cast<uint8_t>(raw);
}

struct Case {
    const char* name;
    float capacitance, resistance, current_offset, noise;
};

static void run(const Case& c) {
    SuperCap super_cap({.can = Can(1)});
    std::mt19937 rng(1);
    std::normal_distribution<float> noise(0, 1);

    // 电流曲线：充电 3 A、放电脉冲 -15 A、静置，周期 6 s
    float charge = 18 * c.capacitance; // 初始 18 V
    const double dt = 0.01;            // 应答 100 Hz
    std::printf("%s: C=%.2f F, ESR=%.3f Ω, 电流偏置 %.2f A, 噪声 %.2f A\n", c.name, c.capacitance, c.resistance, c.current_offset, c.noise);
    for (int k = 0; k < 6000; k++) {
        const double phase = std::fmod(k * dt, 6.0);
        float current = 0;
        if (phase < 3)
            current = 3;
        else if (phase < 4)
            current = -15;
        charge += current * static_cast<float>(dt);
        const float voltage = charge / c.capacitance + c.resistance * current;
        sim_time += dt;

        CanFrame frame{.id = 0x051, .size = 8};
        put(&frame.data[0], 24);
        put(&frame.data[2], 0);
        put(&frame.data[4], voltage + 0.002f * noise(rng));
        put(&frame.data[6], current + c.current_offset + c.noise * noise(rng));
        rx_callback(frame);

        if ((k + 1) % 1000 == 0)
            std::printf("  t=%2.0f s  V=%.2f  C=%.3f F  ESR=%.4f Ω\n", sim_time, voltage, super_cap.capacitance, super_cap.resistance);
    }
}

int main() {
    run({"理想", 6, 0.1f, 0, 0});
    run({"噪声", 6, 0.1f, 0, 0.1f});
    run({"噪声+偏置", 6, 0.1f, 0.1f, 0.1f});
    run({"小容量", 2, 0.2f, 0, 0.1f});
}
