/*
 * 双声部蜂鸣器播放测试（mc02）：依次播放 songs.hpp 中的曲目各一遍，之后静音
 * TIM12 CH2 / PB15 作 80 kHz 载波 PWM-DAC，轮询更新标志，每个载波周期写入一个采样
 * 旋律、伴奏均为 PolyBLEP 方波加衰减包络，权重 0.7 / 0.3，自动增益补偿包络衰减
 * 方案选择过程见 agents/tasks/02-buzzer_timbre/README.md；直接使用 HAL，验证后迁入 lib/bsp
 */

#include "bsp/timer.hpp"
#include "songs.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

// PWM-DAC
static constexpr uint32_t PERIOD = 3250;         // TIM12 时钟 260 MHz / 载波 80 kHz
static constexpr float SAMPLE_DT = 1.0f / 80000; // 采样周期 s，等于载波周期
static constexpr float FADE = 0.02f;             // 播放首尾占空比在 0 与中点间的斜坡时间 s
static constexpr float GAP = 2.0f;               // 曲目间静音 s

// 曲目
static constexpr float UNIT = 0.01f;         // 条目时长单位 s
static constexpr float NOTE_RELEASE = 0.03f; // 松开后淡出时间上限 s

// 自动增益：两声部包络和 A 压缩为 A^(1/GAIN_RATIO)，A = 1 时与直驱方波同响
static constexpr float GAIN_RATIO = 4.0f;
static constexpr float GAIN_MAX = 8.0f;
static constexpr float GAIN_ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float GAIN_RELEASE = 0.15f; // 增益上升时间常数 s
static constexpr int GAIN_EVERY = 8;         // 每 8 个采样（100 µs）更新一次增益

// PolyBLEP 跳变校正：t 为距跳变的相位 [0, 1)，dt 为每采样相位增量
static float poly_blep(float t, const float dt) {
    if (t < dt) {
        t /= dt;
        return t + t - t * t - 1;
    }
    if (t > 1 - dt) {
        t = (t - 1) / dt;
        return t * t + t + t + 1;
    }
    return 0;
}

// 限带方波 ±1，相位 [0, 0.5) 为 1
static float square(const float phase, const float dt) {
    const float half = phase < 0.5f ? phase + 0.5f : phase - 0.5f;
    return (phase < 0.5f ? 1.0f : -1.0f) + poly_blep(phase, dt) - poly_blep(half, dt);
}

// 单声部：限带方波 + 衰减包络，每次 update 推进一个采样
class Voice {
  public:
    struct Config {
        float weight; // 峰值幅度
        float tau_a4; // A4 处衰减时间常数 s，按 sqrt(440/f) 随音高缩放，低音衰减慢
        float floor;  // 余音底，相对峰值
    };

    float amplitude = 0; // 本采样包络幅度，范围 [0, weight]

    Voice(std::span<const VoiceNote> notes, const Config& config);

    // 推进一个采样，返回输出，范围约 [-weight, weight]
    float update();

    bool is_done() const {
        return index_ >= notes_.size();
    }

  private:
    const std::span<const VoiceNote> notes_;
    const Config config_;

    size_t index_ = 0;
    float left_ = 0;                   // 当前条目剩余时间 s
    float phase_ = 0, step_ = 0;       // 相位 [0, 1) 与每采样增量；增量为 0 表示尚未发声
    float decay_ = 0, decay_step_ = 1; // exp(-age/tau) 与每采样衰减系数
    float fade_ = 0, fade_step_ = 0;   // 休止内淡出系数与每采样减量

    void enter();
};

Voice::Voice(const std::span<const VoiceNote> notes, const Config& config)
    : notes_(notes), config_(config) {
    enter();
}

float Voice::update() {
    // 推进条目，剩余时间累加到下一条，误差不累积
    left_ -= SAMPLE_DT;
    while (left_ <= 0 && !is_done()) {
        index_++;
        if (!is_done())
            enter();
    }
    amplitude = 0;
    if (is_done())
        return 0;

    // 包络：衰减到余音底后保持；休止条目内上一个音淡出
    decay_ *= decay_step_;
    amplitude = config_.weight * (config_.floor + (1 - config_.floor) * decay_);
    if (notes_[index_].midi == 0) {
        fade_ = std::max(fade_ - fade_step_, 0.0f);
        amplitude *= fade_;
    }

    // 方波，相位跨音符连续
    phase_ += step_;
    if (phase_ >= 1)
        phase_ -= 1;
    return amplitude * square(phase_, step_);
}

// 进入 index_ 条目
void Voice::enter() {
    const VoiceNote& note = notes_[index_];
    const float len = static_cast<float>(note.len) * UNIT;
    left_ += len;

    // 休止：上一个音在 NOTE_RELEASE 内淡出，不超过休止时长；曲首休止无音可淡出
    if (note.midi == 0) {
        fade_ = step_ > 0 ? 1.0f : 0.0f;
        fade_step_ = SAMPLE_DT / std::min(NOTE_RELEASE, len);
        return;
    }

    // 起音
    const float hz = 440.0f * std::exp2(static_cast<float>(note.midi - 69) / 12.0f);
    const float tau = std::clamp(config_.tau_a4 * std::sqrt(440.0f / hz), 0.3f, 2.0f);
    step_ = hz * SAMPLE_DT;
    decay_ = 1;
    decay_step_ = std::exp(-SAMPLE_DT / tau);
}

// 旋律占大部分摆幅，响度接近单音；伴奏衰减更快、余音底更低，不拖沓
static constexpr Voice::Config MELODY_VOICE = {.weight = 0.7f, .tau_a4 = 0.7f, .floor = 0.6f};
static constexpr Voice::Config BASS_VOICE = {.weight = 0.3f, .tau_a4 = 0.4f, .floor = 0.12f};

// 设置占空比，限幅到 [0, 1]
static void set_duty(const float duty) {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, static_cast<uint32_t>(std::clamp(duty, 0.0f, 1.0f) * PERIOD));
}

// 等待下一个载波周期；CCR 预装载，之后写入的值在下一周期生效，每个采样恰好保持一个载波周期
static void wait_sample() {
    while (!__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE)) {
    }
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);
}

// 占空比在 FADE 内线性变化，避免爆音
static void ramp_duty(const float from, const float to) {
    const int count = static_cast<int>(FADE / SAMPLE_DT);
    for (int i = 1; i <= count; i++) {
        wait_sample();
        set_duty(from + (to - from) * static_cast<float>(i) / static_cast<float>(count));
    }
}

// 播放一首；增益只在旋律发声时更新，旋律休止时保持，伴奏不被放大
static void play(const Song& song) {
    Voice melody(song.melody, MELODY_VOICE);
    Voice bass(song.bass, BASS_VOICE);

    const float exponent = 1 / GAIN_RATIO - 1;
    const float attack = 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / GAIN_ATTACK);
    const float release = 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / GAIN_RELEASE);
    float gain = 1;

    ramp_duty(0, 0.5f);
    for (int i = 0; !melody.is_done() || !bass.is_done(); i++) {
        const float mix = melody.update() + bass.update();

        // 目标增益只与包络有关，包络变化慢，不必每采样计算
        if (i % GAIN_EVERY == 0 && melody.amplitude > 0) {
            const float target = std::min(std::pow(melody.amplitude + bass.amplitude, exponent), GAIN_MAX);
            gain += (target < gain ? attack : release) * (target - gain);
        }

        wait_sample();
        set_duty(0.5f + 0.5f * gain * mix);
    }
    ramp_duty(0.5f, 0);
}

extern "C" void app_main() {
    // TIM12 改为 80 kHz 载波，占空比 0；PWM 输出已由 bsp_init 启动
    __HAL_TIM_SET_PRESCALER(&htim12, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim12, PERIOD - 1);
    set_duty(0);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);

    // 各播一遍，结束后占空比停在 0，蜂鸣器无电流
    for (const Song& song : {NI, ALWAYS_WITH_ME}) {
        play(song);
        Timer::delay(GAP);
    }
    while (true) {
    }
}
