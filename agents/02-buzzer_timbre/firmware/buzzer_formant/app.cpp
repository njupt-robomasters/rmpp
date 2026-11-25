/*
 * 蜂鸣器共振峰对比：同一首曲前 20 s 按下表分段播放，段间静音 1 s，全部结束后循环；每段开始从 Cdc(1) 发送说明
 * 钢琴波表（与 buzzer_piano 相同）上增强最接近谐振区的一根谐波：旋律 3.5 kHz、伴奏 3 kHz 且强度减半，减少两声部拍频
 * 共振峰幅度相对基波；钢琴频谱在谐振区原有谐波约 0.01–0.08，0.3 约对应 +12 dB 以上
 */

#include "agc.hpp"
#include "always_with_me_duo.hpp"
#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <cstdio>
#include <iterator>

static constexpr float SEGMENT = 20.0f; // 每段时长 s
static constexpr float GAP = 1.0f;      // 段间静音 s
static constexpr float FADE = 0.02f;    // 斜坡时间 s
static constexpr float FORMANTS[] = {0, 0.15f, 0.3f, 0.6f, 1.0f};
static constexpr const char* TEXTS[] = {"1 钢琴波表，无共振峰（同 buzzer_piano）", "2 共振峰 0.15", "3 共振峰 0.3", "4 共振峰 0.6", "5 共振峰 1.0"};

static const Cdc cdc(1);

static void report(const char* text) {
    char line[96];
    const int size = std::snprintf(line, sizeof(line), "%s\r\n", text);
    cdc.send(reinterpret_cast<const uint8_t*>(line), size);
}

// 播放一段，自动增益与 buzzer_piano 相同
static void play(Voice& melody, Voice& bass) {
    ramp_duty(0, 0.5f, FADE);
    play_duo_agc(melody, bass, 4.0f, 8.0f, 0.005f, 0.15f, SEGMENT);
    ramp_duty(0.5f, 0, FADE);
    Timer::delay(GAP);
}

extern "C" void app_main() {
    start_carrier();

    while (true) {
        for (int i = 0; i < static_cast<int>(std::size(FORMANTS)); i++) {
            report(TEXTS[i]);
            Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.5f, 0.7f, 0.35f, Wave::PIANO, 0, FORMANTS[i], 3500);
            Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.5f, 0.4f, 0.12f, Wave::PIANO, 0, FORMANTS[i] / 2, 3000);
            play(melody, bass);
        }

        // 对照：方波，与 buzzer_agc 系列同响
        report("6 方波对照");
        Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.5f, 0.7f, 0.35f);
        Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.5f, 0.4f, 0.12f);
        play(melody, bass);
        Timer::delay(2.0f);
    }
}
