/*
 * 2 声部蜂鸣器测试：采样与 80 kHz 载波同步 + PolyBLEP 限带方波 + 放宽余量
 * 中点固定 50%，与 buzzer_duo_bias 对比
 */

#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <iterator>

static constexpr float FADE = 0.02f; // 开始、结束时占空比在 0 与中点间的斜坡时间 s

// 各声部 0.7，同时峰值时少量限幅；旋律余音底提高到 0.45
static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.7f, 0.7f, 0.45f);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.7f, 0.4f, 0.12f);

extern "C" void app_main() {
    start_carrier();
    ramp_duty(0, 0.5f, FADE);

    // 每个载波周期一个采样
    while (!melody.is_done() || !bass.is_done()) {
        const float mix = melody.update() + bass.update();
        wait_sample();
        set_duty(0.5f + 0.5f * mix);
    }

    // 结束后占空比停在 0，蜂鸣器无电流
    ramp_duty(0.5f, 0, FADE);
    while (true) {
    }
}
