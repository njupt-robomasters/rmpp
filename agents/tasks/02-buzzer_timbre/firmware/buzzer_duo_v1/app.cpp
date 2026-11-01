/*
 * 2 声部方波蜂鸣器测试 v1（mc02）：首次上板版本，保留作对照；改进版见 buzzer_duo_sync、buzzer_duo_bias
 * TIM12 CH2 / PB15 设为 80 kHz 载波当 PWM-DAC，主循环按 Timer dt 约 40 kHz 合成采样并写 CCR
 * 音色参数对应 agents/tasks/02-buzzer_timbre/simulate.py 的 7b；直接使用 HAL，验证后迁入 lib/bsp
 */

#include "always_with_me_duo.hpp"
#include "bsp/timer.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>

static constexpr uint32_t PERIOD = 3250;         // TIM12 时钟 260 MHz / 载波 80 kHz
static constexpr float SAMPLE_DT = 1.0f / 40000; // 采样周期 s
static constexpr float UNIT = 0.01f;             // 曲目时长单位 s
static constexpr float RELEASE = 0.03f;          // 松开后线性淡出时间上限 s
static constexpr float FADE = 0.02f;             // 开始、结束时占空比在 0 与中点间的斜坡时间 s

// 单声部：方波 + 衰减包络，按曲目条目推进
class Voice {
  public:
    // weight 为输出幅度；tau_a4 为 A4 处衰减时间常数 s，按 sqrt(440/f) 随音高缩放；floor 为余音底，相对峰值
    Voice(const VoiceNote* notes, size_t count, float weight, float tau_a4, float floor);

    // 推进 dt 秒并返回本采样输出，范围 [-weight, weight]
    float update(float dt);

    bool is_done() const {
        return index_ >= count_;
    }

  private:
    const VoiceNote* const notes_;
    const size_t count_;
    const float weight_, tau_a4_, floor_;

    size_t index_ = 0;
    float left_ = 0;                    // 当前条目剩余时间 s
    float hz_ = 0, tau_ = 1;            // 当前音频率与衰减时间常数
    float phase_ = 0, age_ = 0;         // 方波相位 [0, 1)，起音后经过的时间 s
    float release_ = 0, since_end_ = 0; // 休止条目内的淡出时间与已过时间 s

    void start();
};

Voice::Voice(const VoiceNote* notes, const size_t count, const float weight, const float tau_a4, const float floor)
    : notes_(notes), count_(count), weight_(weight), tau_a4_(tau_a4), floor_(floor) {
    start();
}

float Voice::update(const float dt) {
    // 推进条目，剩余时间累加下一条，误差不累积
    left_ -= dt;
    while (left_ <= 0 && index_ < count_) {
        index_++;
        if (index_ < count_)
            start();
    }
    if (index_ >= count_)
        return 0;

    // 包络：衰减到 floor 后保持；休止条目内上一个音线性淡出
    age_ += dt;
    float amp = floor_ + (1 - floor_) * std::exp(-age_ / tau_);
    if (notes_[index_].midi == 0) {
        since_end_ += dt;
        if (since_end_ >= release_)
            return 0;
        amp *= 1 - since_end_ / release_;
    }

    // 方波，相位跨音符连续
    phase_ += hz_ * dt;
    phase_ -= std::floor(phase_);
    return weight_ * amp * (phase_ < 0.5f ? 1.0f : -1.0f);
}

// 进入 index_ 条目
void Voice::start() {
    const VoiceNote& note = notes_[index_];
    const float len = static_cast<float>(note.len) * UNIT;
    left_ += len;

    // 休止：上一个音在 RELEASE 内淡出，不超过休止时长
    if (note.midi == 0) {
        release_ = std::min(RELEASE, len);
        since_end_ = 0;
        return;
    }

    // 起音：低音衰减慢、高音衰减快
    hz_ = 440.0f * std::exp2(static_cast<float>(note.midi - 69) / 12.0f);
    tau_ = std::clamp(tau_a4_ * std::sqrt(440.0f / hz_), 0.3f, 2.0f);
    age_ = 0;
}

// 旋律与伴奏各占一半幅度；伴奏衰减更快、余音底更低，旋律休止时伴奏已收敛
static Voice melody(ALWAYS_WITH_ME_MELODY, std::size(ALWAYS_WITH_ME_MELODY), 0.5f, 0.7f, 0.35f);
static Voice bass(ALWAYS_WITH_ME_BASS, std::size(ALWAYS_WITH_ME_BASS), 0.5f, 0.4f, 0.12f);

extern "C" void app_main() {
    // TIM12 改为 80 kHz 载波；CCR 预装载由 HAL 配置 PWM 通道时开启，新值在下一载波周期生效
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim12, PERIOD - 1);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    Timer timer;
    float time = 0;   // 播放时间 s
    float ending = 0; // 两声部结束后经过的时间 s
    while (true) {
        if (timer.elapsed() < SAMPLE_DT)
            continue;

        // 按实际 dt 推进，中断延迟只推迟单个采样，音高和节奏不漂移
        const float dt = timer.restart();
        time += dt;
        const float mix = melody.update(dt) + bass.update(dt);

        // 开始与结束时占空比在 0 与中点间斜坡过渡，避免爆音
        float level = std::min(time / FADE, 1.0f);
        if (melody.is_done() && bass.is_done()) {
            ending += dt;
            if (ending >= FADE)
                break;
            level = std::min(level, 1 - ending / FADE);
        }

        const auto ccr = static_cast<uint32_t>(level * (0.5f + 0.5f * mix) * PERIOD);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, ccr);
    }

    // 播放结束，占空比停在 0，蜂鸣器无电流
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
    while (true) {
    }
}
