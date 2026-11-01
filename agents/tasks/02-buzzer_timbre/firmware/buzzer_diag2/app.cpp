/*
 * 低音与复音沙沙声定位，循环播放 4 段，每段 3 s、段间静音 1 s；每段开始从 Cdc(1) 发送说明
 * 1. 低音 F3（175 Hz）方波
 * 2. 同一低音方波经 1 kHz 低通；比 1 干净说明低音沙沙声来自谐振区内的密集高次谐波
 * 3. 和弦 F3 + A4，两声部方波
 * 4. 同一和弦，低音 1 kHz、旋律 4.5 kHz 低通；比 3 干净说明粗糙感来自高次谐波拍频
 * 不衰减（floor 1），两声部各 0.5
 */

#include "always_with_me_duo.hpp"
#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <cstdio>

static constexpr float GAP = 1.0f;               // 段间静音 s
static constexpr float FADE = 0.02f;             // 斜坡时间 s
static constexpr VoiceNote LOW[] = {{53, 300}};  // F3 3 s
static constexpr VoiceNote HIGH[] = {{69, 300}}; // A4 3 s

static const Cdc cdc(1);

static void report(const char* text) {
    char line[96];
    const int size = std::snprintf(line, sizeof(line), "%s\r\n", text);
    cdc.send(reinterpret_cast<const uint8_t*>(line), size);
}

// 播放一段；high_cutoff 为负时只放低音
static void play(const char* text, const float low_cutoff, const float high_cutoff) {
    report(text);
    Voice low(LOW, 1, 0.5f, 1, 1, Wave::SQUARE, low_cutoff);
    Voice high(HIGH, 1, 0.5f, 1, 1, Wave::SQUARE, high_cutoff < 0 ? 0 : high_cutoff);
    const bool has_high = high_cutoff >= 0;

    ramp_duty(0, 0.5f, FADE);
    while (!low.is_done()) {
        float mix = low.update();
        if (has_high)
            mix += high.update();
        wait_sample();
        set_duty(0.5f + 0.5f * mix);
    }
    ramp_duty(0.5f, 0, FADE);
    Timer::delay(GAP);
}

extern "C" void app_main() {
    start_carrier();

    while (true) {
        play("1 低音 F3 方波", 0, -1);
        play("2 低音 F3 方波 + 1 kHz 低通", 1000, -1);
        play("3 和弦 F3 + A4 方波", 0, 0);
        play("4 和弦 F3 + A4 低通（低音 1 kHz，旋律 4.5 kHz）", 1000, 4500);
        Timer::delay(1.0f);
    }
}
