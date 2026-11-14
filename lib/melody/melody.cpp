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
    : config(config),
      dt(1 / config.carrier_hz),
      attack_step(1 - std::exp(-dt * GAIN_EVERY / config.agc.attack)),
      release_step(1 - std::exp(-dt * GAIN_EVERY / config.agc.release)),
      hold(static_cast<uint32_t>(config.agc.hold / dt)),
      ramp_samples(static_cast<uint32_t>(RAMP / dt)) {}

void Melody::play(const Song& song) {
    stop();
    if (song.melody.empty())
        return;
    direct = song.melody[0];
    stage = Stage::DIRECT;
}

void Melody::play_poly(const Song& song) {
    stop();

    // 采样流未开启，可在主循环初始化合成状态
    const float share = song.chords.empty() ? 1.0f : config.melody_share;
    melody.start(song.melody, share, config.melody, config.inner, config.inner_weight, dt);
    chords.start(song.chords, 1 - share, config.chords, config.inner, config.inner_weight, dt);
    gain = 1;
    sample = 0;
    silent = hold; // 开头视为旋律已缺席，前奏按伴奏调节
    ramp = 0;
    stage = Stage::FADE_IN;

    buzzer.start_stream(config.carrier_hz, [this](const std::span<float> duty) { fill(duty); });
}

void Melody::stop() {
    if (stage == Stage::DIRECT || stage == Stage::IDLE)
        buzzer.mute();
    else
        buzzer.stop_stream();

    stage = Stage::IDLE;
    direct = {};
    index = 0;
}

void Melody::update() {
    // 复音：中断内播完并降到 0 后关闭采样流
    if (stage == Stage::ENDED) {
        stop();
        return;
    }
    if (stage != Stage::DIRECT)
        return;

    // 普通：逐音符计时，首个音符立即开始
    if (index > 0 && note_timer.elapsed() < static_cast<float>(direct[index - 1].len) * UNIT)
        return;
    if (index >= direct.size()) {
        stop();
        return;
    }

    // 只在音高切换时操作蜂鸣器
    const uint8_t midi = direct[index].midi;
    if (index == 0 || midi != direct[index - 1].midi) {
        if (midi == 0)
            buzzer.mute();
        else
            buzzer.set_tone(midi_to_hz(midi));
    }
    index++;
    note_timer.restart();
}

// 采样流回调，在 DMA 中断内执行
void Melody::fill(const std::span<float> duty) {
    monitor.begin();
    for (float& d : duty)
        d = next_sample();
    monitor.end();
}

float Melody::next_sample() {
    const float ramp = static_cast<float>(this->ramp) / static_cast<float>(ramp_samples);
    switch (stage) {
    case Stage::FADE_IN:
        if (++this->ramp >= ramp_samples) {
            stage = Stage::POLY;
            this->ramp = 0;
        }
        return 0.5f * ramp;

    case Stage::FADE_OUT:
        if (++this->ramp >= ramp_samples)
            stage = Stage::ENDED;
        return 0.5f * (1 - ramp);

    case Stage::POLY:
        break;

    default:
        return 0;
    }

    // 各组合成
    const bool should_update_gain = sample++ % GAIN_EVERY == 0;
    const float mix = melody.update(should_update_gain, attack_step, release_step) +
                      chords.update(should_update_gain, attack_step, release_step);

    // 总增益：旋律短暂休止时保持，伴奏不会突然变强；缺席超过 hold 时按伴奏调节
    silent = melody.level > 0 ? 0 : std::min(silent + 1, hold);
    const float level = melody.level + chords.level;
    const bool is_held = silent > 0 && silent < hold;
    if (should_update_gain && level > 0 && !is_held) {
        const float target = std::min(std::pow(level, 1 / config.agc.ratio - 1), config.agc.max_gain);
        follow(gain, target, attack_step, release_step);
    }

    if (melody.is_done() && chords.is_done())
        stage = Stage::FADE_OUT;
    return 0.5f + 0.5f * gain * mix;
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

void Melody::VoiceSynth::start(const Voice voice, const Envelope& envelope, const float weight, const float dt) {
    this->voice = voice;
    this->envelope = envelope;
    this->weight = weight;
    this->dt = dt;
    index = 0;
    left = 0;
    midi = 0;
    phase = step = 0;
    decay = 0;
    decay_step = 1;
    fade = fade_step = 0;
    amplitude = 0;
    if (!is_done())
        enter();
}

float Melody::VoiceSynth::update() {
    // 推进条目，剩余时间累加到下一条，误差不累积
    left -= dt;
    while (left <= 0 && !is_done()) {
        index++;
        if (!is_done())
            enter();
    }
    amplitude = 0;
    if (is_done())
        return 0;

    // 包络：衰减到 sustain 后保持；休止内上一个音淡出
    decay *= decay_step;
    amplitude = weight * (envelope.sustain + (1 - envelope.sustain) * decay);
    if (midi == 0) {
        fade = std::max(fade - fade_step, 0.0f);
        amplitude *= fade;
    }

    // 方波，相位跨音符连续
    phase += step;
    if (phase >= 1)
        phase -= 1;
    return amplitude * square(phase, step);
}

// 进入 index 条目
void Melody::VoiceSynth::enter() {
    const Note& note = voice[index];
    const float len = static_cast<float>(note.len) * UNIT;
    left += len;

    // 相邻同音继续发声，不重新起音
    if (note.midi == midi)
        return;

    // 休止：上一个音在 NOTE_RELEASE 内淡出，不超过休止时长
    if (note.midi == 0) {
        midi = 0;
        fade = 1;
        fade_step = dt / std::min(NOTE_RELEASE, len);
        return;
    }

    // 起音：衰减时间常数随音高缩放后限幅
    const float hz = midi_to_hz(note.midi);
    const float tau = std::clamp(envelope.decay * std::sqrt(440.0f / hz), TAU_MIN, TAU_MAX);
    midi = note.midi;
    step = hz * dt;
    decay = 1;
    decay_step = std::exp(-dt / tau);
}

void Melody::PartSynth::start(const Part part, const float share, const Envelope& lead, const Envelope& inner,
                              const float inner_weight, const float dt) {
    count = std::min(part.size(), MAX_VOICES);
    for (std::size_t k = 0; k < count; k++) {
        if (k == 0)
            voices[k].start(part[k], lead, 1, dt);
        else
            voices[k].start(part[k], inner, inner_weight, dt);
    }
    this->share = share;
    gain = share;
    level = 0;
}

float Melody::PartSynth::update(const bool should_update_gain, const float attack_step, const float release_step) {
    float out = 0, square_sum = 0;
    for (std::size_t k = 0; k < count; k++) {
        out += voices[k].update();
        square_sum += voices[k].amplitude * voices[k].amplitude;
    }
    const float raw = std::sqrt(square_sum);

    if (should_update_gain)
        follow(gain, share / std::max(raw, 1.0f), attack_step, release_step);
    level = gain * raw;
    return gain * out;
}

bool Melody::PartSynth::is_done() const {
    return std::all_of(voices, voices + count, [](const VoiceSynth& voice) { return voice.is_done(); });
}
