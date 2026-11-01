/*
 * 2 声部蜂鸣器测试：与 buzzer_soft_lp 相同，但伴奏改用三角波（谐波按 1/n² 衰减），不再低通
 */

#include "agc.hpp"
#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <iterator>

static constexpr float MELODY_CUTOFF = 4500.0f;
static constexpr float RATIO = 4.0f;
static constexpr float MAX_GAIN = 8.0f;
static constexpr float ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float RELEASE = 0.15f; // 增益上升时间常数 s
static constexpr float FADE = 0.02f;    // 开始、结束时占空比在 0 与中点间的斜坡时间 s

static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.65f, 0.7f, 0.45f, Wave::SQUARE, MELODY_CUTOFF);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.35f, 0.4f, 0.12f, Wave::TRIANGLE);

extern "C" void app_main() {
    start_carrier();
    ramp_duty(0, 0.5f, FADE);
    play_duo_agc(melody, bass, RATIO, MAX_GAIN, ATTACK, RELEASE);
    ramp_duty(0.5f, 0, FADE);
    while (true) {
    }
}
