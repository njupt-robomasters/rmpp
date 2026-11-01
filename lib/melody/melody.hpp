#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "bsp/buzzer.hpp"
#include "bsp/monitor.hpp"
#include "bsp/timer.hpp"

// 音符：midi 为 MIDI 音符号（69 为 A4 = 440 Hz），0 为休止；len 单位 10 ms
// 相邻同音条目连续发声，重击之间须有休止
struct Note {
    uint8_t midi;
    uint8_t len;
};

using Voice = std::span<const Note>; // 声部：一条单音序列
using Part = std::span<const Voice>; // 声部组：同时进行的若干声部，第 0 个为主声部

// 曲目，由 tools/midi_to_song.py 生成，见 agents/docs/music.md
struct Song {
    Part melody; // 旋律组，主声部取最高音
    Part chords; // 伴奏组，主声部取最低音；可为空
};

// 板载蜂鸣器播放器
// play 普通模式：直驱方波，只放旋律主声部，用于提示音
// play_poly 复音模式：蜂鸣器作 PWM-DAC，在采样流中断内合成全部声部
class Melody {
  public:
    struct Envelope {
        float decay;   // A4 处衰减时间常数 s，按 sqrt(440/f) 随音高缩放，低音衰减慢
        float sustain; // 衰减后保持的电平，相对起音峰值
    };

    // 自动增益：总电平 A 放大为 A^(1/ratio)，A = 1 时与直驱方波同响
    struct Agc {
        float ratio;    // 越大越响，衰减感越弱
        float max_gain; // 避免把淡出的尾音拉满
        float attack;   // 增益下降时间常数 s
        float release;  // 增益上升时间常数 s
        float hold;     // 旋律休止不超过此时长 s 时增益保持；更长时（前奏、间奏）按伴奏调节
    };

    struct Config {
        float carrier_hz = 32000;                            // 复音模式载波即采样率，取 20~64 kHz 内 1 kHz 的整数倍
        float melody_share = 0.7f;                           // 旋律组占输出摆幅，伴奏组为其余
        Envelope melody = {.decay = 0.7f, .sustain = 0.6f};  // 旋律主声部
        Envelope chords = {.decay = 0.4f, .sustain = 0.12f}; // 伴奏主声部，余音低不拖沓
        Envelope inner = {.decay = 0.15f, .sustain = 0.1f};  // 其余声部，快衰减只在起音时加厚和声
        float inner_weight = 0.5f;                           // 其余声部相对主声部的音量
        Agc agc = {.ratio = 4, .max_gain = 8, .attack = 0.005f, .release = 0.15f, .hold = 0.5f};
    };

    Monitor monitor; // 复音合成中断的频率与占用

    Melody();
    explicit Melody(const Config& config);

    const Config& config() const {
        return config_;
    }

    // 从头播放，打断当前播放；曲目须在播放期间保持有效
    void play(const Song& song);
    void play_poly(const Song& song);

    // 停止播放并静音
    void stop();

    // 主循环约 1 kHz 调用：普通模式到时切换音符，复音模式播完后关闭采样流
    void update();

    bool is_playing() const {
        return stage_ != Stage::IDLE;
    }

  private:
    static constexpr std::size_t MAX_VOICES = 8; // 每组声部上限，与 tools/midi_to_song.py 一致

    // 复音单声部：限带方波 + 衰减包络，每次 update 推进一个采样
    class VoiceSynth {
      public:
        float amplitude = 0; // 本采样包络幅度

        void start(Voice voice, const Envelope& envelope, float weight, float dt);
        float update();

        bool is_done() const {
            return index_ >= voice_.size();
        }

      private:
        Voice voice_;
        Envelope envelope_ = {};
        float weight_ = 0, dt_ = 0;

        std::size_t index_ = 0;
        float left_ = 0;                   // 当前条目剩余时间 s
        uint8_t midi_ = 0;                 // 正在发声的音符，0 为休止
        float phase_ = 0, step_ = 0;       // 相位 [0, 1) 与每采样增量
        float decay_ = 0, decay_step_ = 1; // exp(-age/tau) 与每采样衰减系数
        float fade_ = 0, fade_step_ = 0;   // 休止内淡出系数与每采样减量

        void enter();
    };

    // 复音声部组：组电平按各声部幅度平方和开方，超过 1 时组增益缩小，和弦加厚时本组响度基本不变
    class PartSynth {
      public:
        float level = 0; // 本采样增益后的组电平

        void start(Part part, float share, const Envelope& lead, const Envelope& inner, float inner_weight, float dt);
        float update(bool should_update_gain, float attack_step, float release_step);
        bool is_done() const;

      private:
        VoiceSynth voices_[MAX_VOICES];
        std::size_t count_ = 0;
        float share_ = 0, gain_ = 0;
    };

    // 复音阶段：FADE_IN 占空比从 0 升到中点，FADE_OUT 降回 0，避免爆音；ENDED 等 update 关闭采样流
    enum class Stage { IDLE,
                       DIRECT,
                       FADE_IN,
                       POLY,
                       FADE_OUT,
                       ENDED };

    const Config config_;

    Buzzer buzzer_;
    volatile Stage stage_ = Stage::IDLE; // 复音阶段由中断推进

    // 普通模式进度：index_ 指向下一个音符，timer_ 对当前音符计时
    Voice direct_;
    std::size_t index_ = 0;
    Timer timer_;

    // 复音模式状态，采样流开启后只在中断内修改
    PartSynth melody_, chords_;
    float gain_ = 1;
    uint32_t sample_ = 0; // 已合成的采样数
    uint32_t silent_ = 0; // 旋律组连续无声的采样数
    uint32_t ramp_ = 0;   // 首尾斜坡进度，采样数

    // 由 config_ 换算
    const float dt_;                         // 采样周期 s
    const float attack_step_, release_step_; // 增益每次更新的跟随系数
    const uint32_t hold_, ramp_samples_;     // 采样数

    void fill(std::span<float> duty);
    float next_sample();
};
