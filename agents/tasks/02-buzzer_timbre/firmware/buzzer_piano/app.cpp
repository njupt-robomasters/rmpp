/*
 * 2 声部蜂鸣器测试：钢琴波表，亮表随包络衰减过渡到暗表，对应 simulate.py 的 9f
 * 自动增益压缩比 4，下降 5 ms、上升 150 ms；两声部各 0.5，包络参数与模拟相同
 */

#include "agc.hpp"
#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <iterator>

static constexpr float RATIO = 4.0f;
static constexpr float MAX_GAIN = 8.0f;
static constexpr float ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float RELEASE = 0.15f; // 增益上升时间常数 s
static constexpr float FADE = 0.02f;    // 开始、结束时占空比在 0 与中点间的斜坡时间 s

static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.5f, 0.7f, 0.35f, Wave::PIANO);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.5f, 0.4f, 0.12f, Wave::PIANO);

extern "C" void app_main() {
    start_carrier();
    ramp_duty(0, 0.5f, FADE);
    play_duo_agc(melody, bass, RATIO, MAX_GAIN, ATTACK, RELEASE);
    ramp_duty(0.5f, 0, FADE);
    while (true) {
    }
}
