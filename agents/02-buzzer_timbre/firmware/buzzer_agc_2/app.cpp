/*
 * 2 声部蜂鸣器测试：buzzer_duo_sync 基础上加自动增益，2:1 压缩，保留较多衰减感
 * 调参：RATIO 越大越响、衰减感越弱；MAX_GAIN 限制放大倍数；声部 floor 影响压缩前的长音响度
 */

#include "agc.hpp"
#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <cmath>
#include <iterator>

static constexpr float RATIO = 2.0f;
static constexpr float MAX_GAIN = 8.0f;
static constexpr float FADE = 0.02f; // 开始、结束时占空比在 0 与中点间的斜坡时间 s

// 两声部各 0.5，之和为 1，由自动增益拉到满幅
static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.5f, 0.7f, 0.45f);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.5f, 0.4f, 0.12f);

extern "C" void app_main() {
    start_carrier();
    ramp_duty(0, 0.5f, FADE);
    play_duo_agc(melody, bass, RATIO, MAX_GAIN, 0, 0.05f);
    ramp_duty(0.5f, 0, FADE);
    while (true) {
    }
}
