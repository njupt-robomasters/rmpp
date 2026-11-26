/*
 * 蜂鸣器伴奏强度细调：从曲目 START 秒处开始（之前伴奏较少），每段播放 SEGMENT 秒，段间静音 1 s，全部结束后循环；每段开始从 Cdc(1) 发送说明
 * 上轮 buzzer_mix：旋律 0.8 以上时响度接近单音，方波伴奏 0.2 才勉强听到；本轮在方波伴奏 0.2–0.35 之间细调，并试低通和更高余音底
 * 自动增益只在旋律发声时更新，旋律休止时保持
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

static constexpr float START = 20.0f;   // 曲目起点 s
static constexpr float SEGMENT = 20.0f; // 每段时长 s
static constexpr float GAP = 1.0f;      // 段间静音 s
static constexpr float FADE = 0.02f;    // 斜坡时间 s
static constexpr float RATIO = 4.0f;    // 自动增益压缩比
static constexpr float MAX_GAIN = 8.0f;
static constexpr float ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float RELEASE = 0.15f; // 增益上升时间常数 s

// 一段的配置；旋律为方波、余音底 0.6，伴奏为方波；bass_cutoff 为 0 不低通
struct Segment {
    const char* text;
    float melody_weight;
    float bass_weight, bass_floor, bass_cutoff;
};

static constexpr Segment SEGMENTS[] = {
    {"1 单音方波（音量基准）", 1.0f, 0, 0.12f, 0},
    {"2 旋律 0.8 + 伴奏 0.2（同上轮第 4 段）", 0.8f, 0.2f, 0.12f, 0},
    {"3 旋律 0.7 + 伴奏 0.3", 0.7f, 0.3f, 0.12f, 0},
    {"4 旋律 0.65 + 伴奏 0.35", 0.65f, 0.35f, 0.12f, 0},
    {"5 旋律 0.7 + 伴奏 0.3，伴奏 2 kHz 低通", 0.7f, 0.3f, 0.12f, 2000},
    {"6 旋律 0.7 + 伴奏 0.3，伴奏余音底 0.3", 0.7f, 0.3f, 0.3f, 0},
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
    Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), segment.melody_weight, 0.7f, segment.melody_weight < 1 ? 0.6f : 1.0f);
    Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), segment.bass_weight, 0.4f, segment.bass_floor, Wave::SQUARE, segment.bass_cutoff);

    // 快进到 START，不输出；Debug 下约 1 s
    for (int i = 0; i < static_cast<int>(START / SAMPLE_DT); i++) {
        melody.update();
        bass.update();
    }

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
