/*
 * 蜂鸣器扫频与线性度诊断，循环播放；每段开始时从 Cdc(1) 发送段号和内容，打开串口助手即可对照
 * 第 1 组 扫频：正弦满幅 0.5 ± 0.5，500 Hz 到 6 kHz 共 12 个频率，每个 1 s；最响处即谐振频率
 * 第 2 组 偏置：1 kHz 方波幅度 ±0.2，中点 0.25 / 0.5 / 0.75；驱动线性时三段一样响
 * 第 3 组 幅度：1 kHz 方波中点 0.5，幅度 ±0.5（等于直驱）/ ±0.25 / ±0.125；线性时每段均匀变小约 6 dB
 * newlib-nano 默认不支持浮点 printf，文本只用整数格式
 */

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "pwm_dac.hpp"
#include <cmath>
#include <cstdio>
#include <numbers>

static constexpr float GAP = 0.5f;   // 段间静音 s
static constexpr float FADE = 0.01f; // 每段首尾幅度斜坡 s
static constexpr int SWEEP[] = {500, 750, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 6000};
static constexpr int CENTERS[] = {25, 50, 75}; // 偏置中点，%
static constexpr float AMPLITUDES[] = {0.5f, 0.25f, 0.125f};
static constexpr const char* AMPLITUDE_TEXT[] = {"幅度 ±0.5（等于直驱）", "幅度 ±0.25（应小 6 dB）", "幅度 ±0.125（应小 12 dB）"};

static const Cdc cdc(1);

// 发送一行说明；未枚举时丢弃
static void report(const char* text) {
    char line[96];
    const int size = std::snprintf(line, sizeof(line), "%s\r\n", text);
    cdc.send(reinterpret_cast<const uint8_t*>(line), size);
}

// 播放一段：占空比 center + amplitude × 波形，幅度首尾斜坡；段前占空比斜坡到 center，段后回到 0
static void play(const float hz, const float center, const float amplitude, const bool is_sine, const float seconds) {
    ramp_duty(0, center, 0.02f);

    const float step = hz * SAMPLE_DT;
    const int count = static_cast<int>(seconds / SAMPLE_DT);
    const int fade = static_cast<int>(FADE / SAMPLE_DT);
    float phase = 0;
    for (int i = 0; i < count; i++) {
        phase += step;
        if (phase >= 1)
            phase -= 1;

        float x = 0;
        if (is_sine)
            x = std::sin(2 * std::numbers::pi_v<float> * phase);
        else
            x = square_blep(phase, step);
        const float level = static_cast<float>(std::min({i, count - i, fade})) / static_cast<float>(fade);
        wait_sample();
        set_duty(center + amplitude * level * x);
    }

    ramp_duty(center, 0, 0.02f);
    Timer::delay(GAP);
}

extern "C" void app_main() {
    start_carrier();

    char text[64];
    while (true) {
        report("=== 第 1 组 扫频：正弦满幅，听哪个频率最响 ===");
        for (const int hz : SWEEP) {
            std::snprintf(text, sizeof(text), "%d Hz", hz);
            report(text);
            play(static_cast<float>(hz), 0.5f, 0.5f, true, 1.0f);
        }
        Timer::delay(1.0f);

        report("=== 第 2 组 偏置：1 kHz 方波 ±0.2，线性时三段一样响 ===");
        for (const int center : CENTERS) {
            std::snprintf(text, sizeof(text), "中点 %d%%", center);
            report(text);
            play(1000, static_cast<float>(center) / 100, 0.2f, false, 1.5f);
        }
        Timer::delay(1.0f);

        report("=== 第 3 组 幅度：1 kHz 方波，线性时每段均匀变小 ===");
        for (int i = 0; i < 3; i++) {
            report(AMPLITUDE_TEXT[i]);
            play(1000, 0.5f, AMPLITUDES[i], false, 1.5f);
        }
        Timer::delay(2.0f);
    }
}
