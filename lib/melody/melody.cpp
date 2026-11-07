#include "melody.hpp"
#include <algorithm>
#include <cmath>

static constexpr float UNIT = 0.01f;         // Note::len 单位 s
static constexpr float NOTE_RELEASE = 0.03f; // 松开后淡出上限 s，避免切断时的咔哒声
static constexpr float RAMP = 0.02f;         // 复音首尾占空比在 0 与中点间的斜坡 s，避免爆音
static constexpr float TAU_MIN = 0.3f;       // 按音高缩放后的衰减时间常数下限 s
static constexpr float TAU_MAX = 2.0f;       // 上限 s
static constexpr uint32_t GAIN_EVERY = 8;    // 增益每 8 个采样更新一次，包络变化慢

Melody::Melody(const Config& config)
    : config_(config), dt_(1 / config.carrier_hz),
      attack_step_(1 - std::exp(-dt_ * GAIN_EVERY / config.agc.attack)),
      release_step_(1 - std::exp(-dt_ * GAIN_EVERY / config.agc.release)),
      hold_(static_cast<uint32_t>(config.agc.hold / dt_)), ramp_samples_(static_cast<uint32_t>(RAMP / dt_)) {}

void Melody::play(const Song& song) {
    stop();
    if (song.melody.empty())
        return;
    direct_ = song.melody[0];
    stage_ = Stage::DIRECT;
}

void Melody::play_poly(const Song& song) {
    stop();

    // 采样流未开启，可在主循环初始化合成状态
    const float melody_share = song.chords.empty() ? 1.0f : config_.melody_share;
    melody_.start(song.melody, melody_share, config_.melody, config_.inner, config_.inner_weight, dt_);
    chords_.start(song.chords, 1 - melody_share, config_.chords, config_.inner, config_.inner_weight, dt_);
    gain_ = 1;
    sample_ = 0;
    silent_ = hold_; // 开头视为旋律已缺席，前奏按伴奏调节
    ramp_ = 0;
    stage_ = Stage::FADE_IN;

    buzzer_.start_stream(config_.carrier_hz, [this](const std::span<float> duty) { fill(duty); });
}

void Melody::stop() {
    if (stage_ == Stage::DIRECT || stage_ == Stage::IDLE)
        buzzer_.mute();
    else
        buzzer_.stop_stream();

    stage_ = Stage::IDLE;
    direct_ = {};
    index_ = 0;
}

void Melody::update() {
    // 复音：中断内播完并降到 0 后关闭采样流
    if (stage_ == Stage::ENDED) {
        stop();
        return;
    }
    if (stage_ != Stage::DIRECT)
        return;

    // 普通：逐音符计时，首个音符立即开始
    if (index_ > 0 && timer_.elapsed() < static_cast<float>(direct_[index_ - 1].len) * UNIT)
        return;
    if (index_ >= direct_.size()) {
        stop();
        return;
    }

    // 只在音高切换时操作蜂鸣器
    const uint8_t midi = direct_[index_].midi;
    if (index_ == 0 || midi != direct_[index_ - 1].midi) {
        if (midi == 0)
            buzzer_.mute();
        else
            buzzer_.set_tone(midi_to_hz(midi));
    }
    index_++;
    timer_.restart();
}

// 采样流回调，在 DMA 中断内执行
void Melody::fill(const std::span<float> duty) {
    monitor.begin();
    for (float& d : duty)
        d = next_sample();
    monitor.end();
}

float Melody::next_sample() {
    const float ramp = static_cast<float>(ramp_) / static_cast<float>(ramp_samples_);
    switch (stage_) {
    case Stage::FADE_IN:
        if (++ramp_ >= ramp_samples_) {
            stage_ = Stage::POLY;
            ramp_ = 0;
        }
        return 0.5f * ramp;

    case Stage::FADE_OUT:
        if (++ramp_ >= ramp_samples_)
            stage_ = Stage::ENDED;
        return 0.5f * (1 - ramp);

    case Stage::POLY:
        break;

    default:
        return 0;
    }

    // 各组合成
    const bool should_update_gain = sample_++ % GAIN_EVERY == 0;
    const float mix = melody_.update(should_update_gain, attack_step_, release_step_) +
                      chords_.update(should_update_gain, attack_step_, release_step_);

    // 总增益：旋律短暂休止时保持，伴奏不会突然变强；缺席超过 hold 时按伴奏调节
    silent_ = melody_.level > 0 ? 0 : std::min(silent_ + 1, hold_);
    const float level = melody_.level + chords_.level;
    const bool is_held = silent_ > 0 && silent_ < hold_;
    if (should_update_gain && level > 0 && !is_held) {
        const float target = std::min(std::pow(level, 1 / config_.agc.ratio - 1), config_.agc.max_gain);
        follow(gain_, target, attack_step_, release_step_);
    }

    if (melody_.is_done() && chords_.is_done())
        stage_ = Stage::FADE_OUT;
    return 0.5f + 0.5f * gain_ * mix;
}

void Melody::VoiceSynth::start(const Voice voice, const Envelope& envelope, const float weight, const float dt) {
    voice_ = voice;
    envelope_ = envelope;
    weight_ = weight;
    dt_ = dt;
    index_ = 0;
    left_ = 0;
    midi_ = 0;
    phase_ = step_ = 0;
    decay_ = 0;
    decay_step_ = 1;
    fade_ = fade_step_ = 0;
    amplitude = 0;
    if (!is_done())
        enter();
}

float Melody::VoiceSynth::update() {
    // 推进条目，剩余时间累加到下一条，误差不累积
    left_ -= dt_;
    while (left_ <= 0 && !is_done()) {
        index_++;
        if (!is_done())
            enter();
    }
    amplitude = 0;
    if (is_done())
        return 0;

    // 包络：衰减到 sustain 后保持；休止内上一个音淡出
    decay_ *= decay_step_;
    amplitude = weight_ * (envelope_.sustain + (1 - envelope_.sustain) * decay_);
    if (midi_ == 0) {
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
void Melody::VoiceSynth::enter() {
    const Note& note = voice_[index_];
    const float len = static_cast<float>(note.len) * UNIT;
    left_ += len;

    // 相邻同音继续发声，不重新起音
    if (note.midi == midi_)
        return;

    // 休止：上一个音在 NOTE_RELEASE 内淡出，不超过休止时长
    if (note.midi == 0) {
        midi_ = 0;
        fade_ = 1;
        fade_step_ = dt_ / std::min(NOTE_RELEASE, len);
        return;
    }

    // 起音：衰减时间常数随音高缩放后限幅
    const float hz = midi_to_hz(note.midi);
    const float tau = std::clamp(envelope_.decay * std::sqrt(440.0f / hz), TAU_MIN, TAU_MAX);
    midi_ = note.midi;
    step_ = hz * dt_;
    decay_ = 1;
    decay_step_ = std::exp(-dt_ / tau);
}

void Melody::PartSynth::start(const Part part, const float share, const Envelope& lead, const Envelope& inner,
                              const float inner_weight, const float dt) {
    count_ = std::min(part.size(), MAX_VOICES);
    for (std::size_t k = 0; k < count_; k++) {
        if (k == 0)
            voices_[k].start(part[k], lead, 1, dt);
        else
            voices_[k].start(part[k], inner, inner_weight, dt);
    }
    share_ = share;
    gain_ = share;
    level = 0;
}

float Melody::PartSynth::update(const bool should_update_gain, const float attack_step, const float release_step) {
    float out = 0, square_sum = 0;
    for (std::size_t k = 0; k < count_; k++) {
        out += voices_[k].update();
        square_sum += voices_[k].amplitude * voices_[k].amplitude;
    }
    const float raw = std::sqrt(square_sum);

    if (should_update_gain)
        follow(gain_, share_ / std::max(raw, 1.0f), attack_step, release_step);
    level = gain_ * raw;
    return gain_ * out;
}

bool Melody::PartSynth::is_done() const {
    return std::all_of(voices_, voices_ + count_, [](const VoiceSynth& voice) { return voice.is_done(); });
}

float Melody::midi_to_hz(const uint8_t midi) {
    return 440.0f * std::exp2((static_cast<float>(midi) - 69.0f) / 12.0f);
}

// 增益平滑跟随：下降快、上升慢
void Melody::follow(float& gain, const float target, const float attack_step, const float release_step) {
    gain += (target < gain ? attack_step : release_step) * (target - gain);
}

// PolyBLEP 跳变校正：t 为距跳变的相位 [0, 1)，dt 为每采样相位增量
float Melody::poly_blep(float t, const float dt) {
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

// 限带方波 ±1，相位 [0, 0.5) 为 1；抑制折回到可听频段的杂音
float Melody::square(const float phase, const float dt) {
    const float half = phase < 0.5f ? phase + 0.5f : phase - 0.5f;
    return (phase < 0.5f ? 1.0f : -1.0f) + poly_blep(phase, dt) - poly_blep(half, dt);
}
