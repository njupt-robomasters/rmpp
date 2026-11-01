#pragma once

// 钢琴波表：亮表为钢琴平均谐波比，暗表第 h 次谐波再乘 0.6^(h-1)；零相位，各自峰值归一化到 1
// 谐波比来自 github.com/xzqiaochu/piano，对应 agents/tasks/02-buzzer_timbre/simulate.py 的 9f
// 最高音 C7 的 15 次谐波约 31 kHz，低于 80 kHz 采样的 40 kHz 奈奎斯特频率，不按音高分表

#include <cmath>
#include <iterator>
#include <numbers>

inline constexpr int PIANO_SIZE = 256;
inline constexpr float PIANO_HARMONICS[] = {1, 0.340f, 0.102f, 0.085f, 0.070f, 0.065f, 0.028f, 0.085f, 0.011f, 0.030f, 0.010f, 0.014f, 0.012f, 0.013f, 0.004f};

// 末尾多存 1 点等于首点，线性插值不必回绕
struct PianoTables {
    float bright[PIANO_SIZE + 1];
    float dark[PIANO_SIZE + 1];
    float sine[PIANO_SIZE + 1]; // 共振峰用
};

inline PianoTables make_piano_tables() {
    PianoTables tables{};
    for (float* table : {tables.bright, tables.dark}) {
        const float rolloff = table == tables.bright ? 1.0f : 0.6f;
        float peak = 0;
        for (int i = 0; i < PIANO_SIZE; i++) {
            float y = 0, gain = 1;
            for (int h = 1; h <= static_cast<int>(std::size(PIANO_HARMONICS)); h++) {
                y += PIANO_HARMONICS[h - 1] * gain * std::sin(2 * std::numbers::pi_v<float> * static_cast<float>(h * i) / PIANO_SIZE);
                gain *= rolloff;
            }
            table[i] = y;
            peak = std::fmax(peak, std::fabs(y));
        }
        for (int i = 0; i < PIANO_SIZE; i++)
            table[i] /= peak;
        table[PIANO_SIZE] = table[0];
    }
    for (int i = 0; i <= PIANO_SIZE; i++)
        tables.sine[i] = std::sin(2 * std::numbers::pi_v<float> * static_cast<float>(i) / PIANO_SIZE);
    return tables;
}

// 启动时生成
inline const PianoTables PIANO_TABLES = make_piano_tables();

// 相位 [0, 1) 处的正弦
inline float sine_wave(const float phase) {
    const float pos = phase * PIANO_SIZE;
    const int i = static_cast<int>(pos);
    const float* const s = PIANO_TABLES.sine;
    return s[i] + (pos - static_cast<float>(i)) * (s[i + 1] - s[i]);
}

// 相位 [0, 1) 处的波形，bright 为亮表权重 [0, 1]，其余为暗表
inline float piano_wave(const float phase, const float bright) {
    const float pos = phase * PIANO_SIZE;
    const int i = static_cast<int>(pos);
    const float frac = pos - static_cast<float>(i);
    const float* const b = PIANO_TABLES.bright;
    const float* const d = PIANO_TABLES.dark;
    const float y_bright = b[i] + frac * (b[i + 1] - b[i]);
    const float y_dark = d[i] + frac * (d[i + 1] - d[i]);
    return y_dark + bright * (y_bright - y_dark);
}
