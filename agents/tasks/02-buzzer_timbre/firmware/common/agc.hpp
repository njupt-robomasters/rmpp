#pragma once

// 两声部自动增益播放：总包络幅度 A 压缩为 A^(1/ratio)，A = 1 时满幅，即与直驱方波同响

#include "pwm_dac.hpp"
#include "voice.hpp"
#include <algorithm>
#include <cmath>

// ratio 为 INFINITY 时总幅度恒为满幅；max_gain 限制放大倍数，避免把松开淡出也拉满
// attack、release 为增益下降、上升的时间常数 s；attack 为 0 时立即下降，大于 0 时起音可能短暂限幅
// seconds 为最长播放时间 s，分段对比用；两声部 weight 之和应为 1
inline void play_duo_agc(Voice& melody, Voice& bass, const float ratio, const float max_gain, const float attack, const float release,
                         const float seconds = INFINITY) {
    static constexpr int GAIN_EVERY = 8; // 每 8 个采样（100 µs）重算一次目标增益

    const float exponent = 1 / ratio - 1;
    const float down = attack > 0 ? 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / attack) : 1.0f;
    const float up = 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / release);
    float gain = 1;
    const int count = std::isinf(seconds) ? -1 : static_cast<int>(seconds / SAMPLE_DT);
    for (int i = 0; (!melody.is_done() || !bass.is_done()) && i != count; i++) {
        const float mix = melody.update() + bass.update();

        // 目标增益只与包络有关，包络变化慢，不必每采样计算
        if (i % GAIN_EVERY == 0) {
            const float level = melody.amplitude() + bass.amplitude();
            float target = max_gain;
            if (level > 0)
                target = std::min(std::pow(level, exponent), max_gain);
            gain += (target < gain ? down : up) * (target - gain);
        }

        wait_sample();
        set_duty(0.5f + 0.5f * gain * mix);
    }
}
