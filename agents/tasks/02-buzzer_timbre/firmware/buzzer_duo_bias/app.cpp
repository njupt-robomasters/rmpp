/*
 * 2 声部蜂鸣器测试：在 buzzer_duo_sync 基础上使用动态偏置
 * 占空比 = 偏置 + 信号，偏置跟随两声部包络幅度之和，输出范围 [0, 2 × 偏置]；休止时占空比自然回到 0
 */

#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <cmath>
#include <iterator>

static constexpr float BIAS_TAU = 0.002f; // 偏置跟随时间常数 s，过快会把包络变化直接变成哔声

static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.7f, 0.7f, 0.45f);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.7f, 0.4f, 0.12f);

extern "C" void app_main() {
    start_carrier();

    const float alpha = 1 - std::exp(-SAMPLE_DT / BIAS_TAU);
    float bias = 0; // 占空比偏置
    while (!melody.is_done() || !bass.is_done()) {
        const float mix = melody.update() + bass.update();

        // 信号幅度（占空比单位）为 0.5 × mix，偏置取幅度包络并限制在 0.5 以内
        const float target = std::min(0.5f * (melody.amplitude() + bass.amplitude()), 0.5f);
        bias += alpha * (target - bias);

        wait_sample();
        set_duty(bias + 0.5f * mix);
    }

    set_duty(0);
    while (true) {
    }
}
