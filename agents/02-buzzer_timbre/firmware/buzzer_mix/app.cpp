/*
 * 蜂鸣器旋律为主对比：同一首曲前 20 s 按 SEGMENTS 分段播放，段间静音 1 s，全部结束后循环；每段开始从 Cdc(1) 发送说明
 * 旋律方波占大部分摆幅，伴奏只作点缀；自动增益只在旋律发声时更新，旋律休止时保持，伴奏不会被放大
 */

#include "always_with_me_duo.hpp"
#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

static constexpr float SEGMENT = 20.0f; // 每段时长 s
static constexpr float GAP = 1.0f;      // 段间静音 s
static constexpr float FADE = 0.02f;    // 斜坡时间 s
static constexpr float RATIO = 4.0f;    // 自动增益压缩比
static constexpr float MAX_GAIN = 8.0f;
static constexpr float ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float RELEASE = 0.15f; // 增益上升时间常数 s

// 一段的配置；旋律为方波，melody_floor 为 1 时不衰减
struct Segment {
    const char* text;
    float melody_weight, melody_floor;
    float bass_weight;
    Wave bass_wave;
};

static constexpr Segment SEGMENTS[] = {
    {"1 单音方波直驱（音量基准）", 1.0f, 1.0f, 0, Wave::SQUARE},
    {"2 双声部方波 0.5 / 0.5", 0.5f, 0.35f, 0.5f, Wave::SQUARE},
    {"3 方波旋律 0.8 + 钢琴伴奏 0.2", 0.8f, 0.6f, 0.2f, Wave::PIANO},
    {"4 方波旋律 0.8 + 方波伴奏 0.2", 0.8f, 0.6f, 0.2f, Wave::SQUARE},
    {"5 方波旋律 0.9 + 钢琴伴奏 0.1", 0.9f, 0.6f, 0.1f, Wave::PIANO},
    {"6 方波旋律 0.8 不衰减 + 钢琴伴奏 0.2", 0.8f, 1.0f, 0.2f, Wave::PIANO},
};

static const Cdc cdc(1);

static void report(const char* text) {
    char line[96];
    const int size = std::snprintf(line, sizeof(line), "%s\r\n", text);
    cdc.send(reinterpret_cast<const uint8_t*>(line), size);
}

// 播放一段 SEGMENT 秒
static void play(const Segment& segment) {
    report(segment.text);
    Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), segment.melody_weight, 0.7f, segment.melody_floor);
    Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), segment.bass_weight, 0.4f, 0.12f, segment.bass_wave);

    const float exponent = 1 / RATIO - 1;
    const float down = 1 - std::exp(-SAMPLE_DT * 8 / ATTACK);
    const float up = 1 - std::exp(-SAMPLE_DT * 8 / RELEASE);
    const int count = static_cast<int>(SEGMENT / SAMPLE_DT);
    float gain = 1;

    ramp_duty(0, 0.5f, FADE);
    for (int i = 0; i < count; i++) {
        const float mix = melody.update() + bass.update();

        // 每 8 个采样更新一次增益；旋律休止时保持
        if (i % 8 == 0 && melody.amplitude() > 0) {
            const float level = melody.amplitude() + bass.amplitude();
            const float target = std::min(std::pow(level, exponent), MAX_GAIN);
            gain += (target < gain ? down : up) * (target - gain);
        }

        wait_sample();
        set_duty(0.5f + 0.5f * gain * mix);
    }
    ramp_duty(0.5f, 0, FADE);
    Timer::delay(GAP);
}

extern "C" void app_main() {
    start_carrier();

    while (true) {
        for (const Segment& segment : SEGMENTS)
            play(segment);
        Timer::delay(2.0f);
    }
}
