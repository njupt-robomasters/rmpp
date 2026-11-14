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
  private:
    static constexpr std::size_t MAX_VOICES = 8; // 每组声部上限，与 tools/midi_to_song.py 一致

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

  private:
    // 复音单声部：限带方波 + 衰减包络，每次 update 推进一个采样
    class VoiceSynth {
      public:
        float amplitude = 0; // 本采样包络幅度

      private:
        Envelope envelope = {};
        float weight = 0, dt = 0;
        std::size_t index = 0;
        float left = 0;                  // 当前条目剩余时间 s
        uint8_t midi = 0;                // 正在发声的音符，0 为休止
        float phase = 0, step = 0;       // 相位 [0, 1) 与每采样增量
        float decay = 0, decay_step = 1; // exp(-age/tau) 与每采样衰减系数
        float fade = 0, fade_step = 0;   // 休止内淡出系数与每采样减量
        Voice voice;

      public:
        void start(Voice voice, const Envelope& envelope, float weight, float dt);

        float update();

        bool is_done() const {
            return index >= voice.size();
        }

      private:
        void enter();
    };

    // 复音声部组：组电平按各声部幅度平方和开方，超过 1 时组增益缩小，和弦加厚时本组响度基本不变
    class PartSynth {
      public:
        float level = 0; // 本采样增益后的组电平

      private:
        std::size_t count = 0;
        float share = 0, gain = 0;
        VoiceSynth voices[MAX_VOICES];

      public:
        void start(Part part, float share, const Envelope& lead, const Envelope& inner, float inner_weight, float dt);

        float update(bool should_update_gain, float attack_step, float release_step);

        bool is_done() const;
    };

    // 复音阶段：FADE_IN 占空比从 0 升到中点，FADE_OUT 降回 0，避免爆音；ENDED 等 update 关闭采样流
    enum class Stage { IDLE,
                       DIRECT,
                       FADE_IN,
                       POLY,
                       FADE_OUT,
                       ENDED };

  public:
    Monitor monitor; // 复音合成中断的频率与占用

  private:
    const Config config;

    // 由 config 换算
    const float dt;                        // 采样周期 s
    const float attack_step, release_step; // 增益每次更新的跟随系数
    const uint32_t hold, ramp_samples;     // 采样数
    Buzzer buzzer;
    volatile Stage stage = Stage::IDLE; // 复音阶段由中断推进
    std::size_t index = 0;
    float gain = 1;
    uint32_t sample = 0; // 已合成的采样数
    uint32_t silent = 0; // 旋律组连续无声的采样数
    uint32_t ramp = 0;   // 首尾斜坡进度，采样数

    // 普通模式进度：index 指向下一个音符，note_timer 对当前音符计时
    Voice direct;

    // 复音模式状态，采样流开启后只在中断内修改
    PartSynth melody, chords;
    Timer note_timer;

  public:
    explicit Melody(const Config& config);

    // 从头播放，打断当前播放；曲目须在播放期间保持有效
    void play(const Song& song);

    void play_poly(const Song& song);

    // 停止播放并静音
    void stop();

    // 主循环约 1 kHz 调用：普通模式到时切换音符，复音模式播完后关闭采样流
    void update();

    bool is_playing() const {
        return stage != Stage::IDLE;
    }

  private:
    void fill(std::span<float> duty);

    float next_sample();

    static float midi_to_hz(uint8_t midi);

    // 增益平滑跟随：下降快、上升慢
    static void follow(float& gain, float target, float attack_step, float release_step);

    // PolyBLEP 跳变校正：t 为距跳变的相位 [0, 1)，dt 为每采样相位增量
    static float poly_blep(float t, float dt);

    // 限带方波 ±1，相位 [0, 0.5) 为 1；抑制折回到可听频段的杂音
    static float square(float phase, float dt);
};
