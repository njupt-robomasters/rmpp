/*
 * 2 声部蜂鸣器测试：伴奏方波经 1 kHz 低通，削弱落入 3–4 kHz 谐振区的密集高次谐波；旋律方波经 4.5 kHz 低通，减少两声部高次谐波拍频
 * 平滑自动增益：压缩比 4，下降 5 ms、上升 150 ms
 * 调参：BASS_CUTOFF 越低伴奏越干净也越轻；RATIO 越大越响、衰减感越弱
 */

#include "agc.hpp"
#include "always_with_me_duo.hpp"
#include "pwm_dac.hpp"
#include "voice.hpp"
#include <iterator>

static constexpr float MELODY_CUTOFF = 4500.0f;
static constexpr float BASS_CUTOFF = 1000.0f;
static constexpr float RATIO = 4.0f;
static constexpr float MAX_GAIN = 8.0f;
static constexpr float ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float RELEASE = 0.15f; // 增益上升时间常数 s
static constexpr float FADE = 0.02f;    // 开始、结束时占空比在 0 与中点间的斜坡时间 s

// 旋律 0.65、伴奏 0.35，幅度让给旋律
static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.65f, 0.7f, 0.45f, Wave::SQUARE, MELODY_CUTOFF);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.35f, 0.4f, 0.12f, Wave::SQUARE, BASS_CUTOFF);

extern "C" void app_main() {
    start_carrier();
    ramp_duty(0, 0.5f, FADE);
    play_duo_agc(melody, bass, RATIO, MAX_GAIN, ATTACK, RELEASE);
    ramp_duty(0.5f, 0, FADE);
    while (true) {
    }
}
