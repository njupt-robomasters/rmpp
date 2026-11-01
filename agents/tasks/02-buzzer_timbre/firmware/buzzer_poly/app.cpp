/*
 * 多声部蜂鸣器播放测试（mc02）：完整播放一首后静音，曲目由子目录工程的 CMake 定义 SONG 指定（songs.hpp 中的常量名）
 * 旋律组（右手）与伴奏组（左手）不限音数，槽 0 为主线，其余为内声部；声部分配见 agents/tasks/02-buzzer_timbre/gen_poly.py
 * 各声部为 PolyBLEP 方波加衰减包络；组增益按平方和归一，保持旋律 0.7 / 伴奏 0.3；总自动增益补偿包络衰减
 * TIM12 CH2 / PB15 作 PWM-DAC，轮询更新标志，每个载波周期写入一个采样；每首结束后从 Cdc(1) 打印计算负载
 * 方案选择过程见 agents/tasks/02-buzzer_timbre/README.md；直接使用 HAL，验证后迁入 lib/bsp
 */

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "songs.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <span>

// PWM-DAC：采样率等于载波频率；载波须高于听觉范围，建议不低于 30 kHz；40 kHz 待与 80 kHz 对比
static constexpr float CARRIER_HZ = 40000;
static constexpr float SAMPLE_DT = 1 / CARRIER_HZ; // 采样周期 s
static constexpr float FADE = 0.02f;               // 播放首尾占空比在 0 与中点间的斜坡时间 s
static constexpr float GAP = 1.0f;                 // 上电后开始播放前的静音 s

// 曲目：SONG 由工程 CMake 定义
#define STRINGIFY(x) #x
#define TO_STRING(x) STRINGIFY(x)
static constexpr const char* SONG_NAME = TO_STRING(SONG);
static constexpr float UNIT = 0.01f;         // 条目时长单位 s
static constexpr float NOTE_RELEASE = 0.03f; // 松开后淡出时间上限 s

// 增益平滑：组增益与总自动增益共用；总电平 A 压缩为 A^(1/GAIN_RATIO)，A = 1 时与直驱方波同响
static constexpr float GAIN_RATIO = 4.0f;
static constexpr float GAIN_MAX = 8.0f;
static constexpr float GAIN_ATTACK = 0.005f; // 增益下降时间常数 s
static constexpr float GAIN_RELEASE = 0.15f; // 增益上升时间常数 s
static constexpr int GAIN_EVERY = 8;         // 每 8 个采样更新一次增益
static constexpr float GAIN_HOLD = 0.5f;     // 旋律休止不超过此时长 s 时总增益保持，更长时按伴奏调节

// 增益每次更新的跟随系数
static const float ATTACK_STEP = 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / GAIN_ATTACK);
static const float RELEASE_STEP = 1 - std::exp(-SAMPLE_DT * GAIN_EVERY / GAIN_RELEASE);

// 增益平滑跟随：下降快、上升慢
static void follow(float& gain, const float target) {
    gain += (target < gain ? ATTACK_STEP : RELEASE_STEP) * (target - gain);
}

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

// 一组声部：组电平按各幅度平方和开方计算，超过 1 时组增益缩小，和弦加厚时本组响度基本不变
class Group {
  public:
    float level = 0; // 本采样增益后的组电平

    // share 为本组占整体摆幅的份额
    explicit Group(float share)
        : share_(share), gain_(share) {}

    // 加入槽位：槽 0 用 lead，其余用 inner，weight 为组内相对权重
    void add(std::span<const std::span<const VoiceNote>> slots, const Voice::Config& lead, const Voice::Config& inner);

    // 推进一个采样并返回组输出；should_update_gain 为 true 时更新组增益
    float update(bool should_update_gain);

    bool is_done() const;

  private:
    const float share_;

    std::optional<Voice> voices_[MAX_SLOTS];
    size_t count_ = 0;
    float gain_;
};

void Group::add(const std::span<const std::span<const VoiceNote>> slots, const Voice::Config& lead, const Voice::Config& inner) {
    for (size_t k = 0; k < slots.size(); k++)
        voices_[count_++].emplace(slots[k], k == 0 ? lead : inner);
}

float Group::update(const bool should_update_gain) {
    float out = 0, square_sum = 0;
    for (size_t k = 0; k < count_; k++) {
        out += voices_[k]->update();
        square_sum += voices_[k]->amplitude * voices_[k]->amplitude;
    }
    const float raw = std::sqrt(square_sum);

    if (should_update_gain)
        follow(gain_, share_ / std::max(raw, 1.0f));
    level = gain_ * raw;
    return gain_ * out;
}

bool Group::is_done() const {
    return std::all_of(voices_, voices_ + count_, [](const std::optional<Voice>& voice) { return voice->is_done(); });
}

// 旋律主线衰减慢、余音底高；伴奏主线衰减快、余音底低，不拖沓
// 内声部权重 0.5 且快衰减：突出主线，只在起音时加厚和声，减少持续拍频
static constexpr Voice::Config MELODY_LEAD = {.weight = 1, .tau_a4 = 0.7f, .floor = 0.6f};
static constexpr Voice::Config CHORD_LEAD = {.weight = 1, .tau_a4 = 0.4f, .floor = 0.12f};
static constexpr Voice::Config INNER = {.weight = 0.5f, .tau_a4 = 0.15f, .floor = 0.1f};
static constexpr float MELODY_SHARE = 0.7f, CHORD_SHARE = 0.3f;

static const Cdc cdc(1);
static uint32_t period = 0; // ARR + 1，由定时器时钟与 CARRIER_HZ 确定

// 设置占空比，限幅到 [0, 1]
static void set_duty(const float duty) {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, static_cast<uint32_t>(std::clamp(duty, 0.0f, 1.0f) * static_cast<float>(period)));
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

// 播放一首
// 总增益：旋律短暂休止时保持，伴奏不会突然变强；旋律缺席超过 GAIN_HOLD（前奏、间奏）时按伴奏调节
static void play(const char* name, const Song& song) {
    // 旋律组与伴奏组分别归一
    Group melody(MELODY_SHARE);
    Group chords(CHORD_SHARE);
    melody.add(song.melody, MELODY_LEAD, INNER);
    chords.add(song.chords, CHORD_LEAD, INNER);

    const float exponent = 1 / GAIN_RATIO - 1;
    const auto hold = static_cast<uint32_t>(GAIN_HOLD / SAMPLE_DT);
    float gain = 1;
    uint32_t silent = hold; // 旋律组连续无声的采样数；初始视为已缺席，前奏按伴奏调节

    // 负载：每采样计算所用 CPU 周期（含期间中断）占载波周期的比例；超过一个周期即丢采样
    const auto budget = static_cast<uint32_t>(static_cast<float>(SystemCoreClock) * SAMPLE_DT);
    uint64_t busy_sum = 0;
    uint32_t busy_max = 0, overruns = 0, count = 0;

    ramp_duty(0, 0.5f);
    for (; !melody.is_done() || !chords.is_done(); count++) {
        const uint32_t begin = DWT->CYCCNT;
        const bool should_update_gain = count % GAIN_EVERY == 0;
        const float mix = melody.update(should_update_gain) + chords.update(should_update_gain);
        silent = melody.level > 0 ? 0 : std::min(silent + 1, hold);
        const float level = melody.level + chords.level;
        const bool is_held = silent > 0 && silent < hold; // 旋律短暂休止
        if (should_update_gain && level > 0 && !is_held)
            follow(gain, std::min(std::pow(level, exponent), GAIN_MAX));

        const uint32_t busy = DWT->CYCCNT - begin;
        busy_sum += busy;
        busy_max = std::max(busy_max, busy);
        if (busy > budget)
            overruns++;
        wait_sample();
        set_duty(0.5f + 0.5f * gain * mix);
    }
    ramp_duty(0.5f, 0);

    // 打印负载；newlib-nano 不支持浮点格式，按 0.1% 整数输出
    const auto average = static_cast<unsigned long>(busy_sum * 1000 / (static_cast<uint64_t>(budget) * count));
    const auto peak = static_cast<unsigned long>(static_cast<uint64_t>(busy_max) * 1000 / budget);
    char text[128];
    const int size = std::snprintf(text, sizeof(text), "%s 负载 平均 %lu.%lu%% 最大 %lu.%lu%% 超时 %lu 次\r\n", name, average / 10, average % 10,
                                   peak / 10, peak % 10, static_cast<unsigned long>(overruns));
    cdc.send(reinterpret_cast<const uint8_t*>(text), size);
}

extern "C" void app_main() {
    // TIM12 位于 APB1，分频不为 1 时定时器时钟为 PCLK1 × 2；PWM 输出已由 bsp_init 启动
    period = static_cast<uint32_t>(std::lround(static_cast<float>(HAL_RCC_GetPCLK1Freq() * 2) / CARRIER_HZ));
    __HAL_TIM_SET_PRESCALER(&htim12, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim12, period - 1);
    set_duty(0);
    HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);

    // 播一遍，结束后占空比停在 0，蜂鸣器无电流
    Timer::delay(GAP);
    play(SONG_NAME, SONG);
    while (true) {
    }
}
