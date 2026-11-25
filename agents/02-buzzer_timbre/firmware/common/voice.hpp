#pragma once

// 单声部：PolyBLEP 方波、三角波或钢琴波表 + 衰减包络 + 可选一阶低通，每次 update 推进一个采样（SAMPLE_DT）

#include "always_with_me_duo.hpp"
#include "piano.hpp"
#include "pwm_dac.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

enum class Wave { SQUARE,
                  TRIANGLE,
                  PIANO };

class Voice {
  public:
    // weight 为峰值幅度；tau_a4 为 A4 处衰减时间常数 s，按 sqrt(440/f) 随音高缩放；floor 为余音底，相对峰值
    // cutoff 为输出一阶低通截止频率 Hz，0 为不滤波
    // formant 为钢琴波表共振峰幅度，相对基波；增强最接近 formant_hz 的一根谐波，落入蜂鸣器谐振区以提响
    Voice(const VoiceNote* notes, size_t count, float weight, float tau_a4, float floor, Wave wave = Wave::SQUARE, float cutoff = 0,
          float formant = 0, float formant_hz = 3500)
        : notes_(notes), count_(count), weight_(weight), tau_a4_(tau_a4), floor_(floor), wave_(wave),
          lowpass_(cutoff > 0 ? 1 - std::exp(-2 * std::numbers::pi_v<float> * cutoff * SAMPLE_DT) : 1),
          formant_(formant), formant_hz_(formant_hz) {
        start();
    }

    // 推进一个采样并返回输出，范围约 [-weight, weight]
    float update() {
        // 推进条目，剩余时间累加到下一条，误差不累积
        left_ -= SAMPLE_DT;
        while (left_ <= 0 && index_ < count_) {
            index_++;
            if (index_ < count_)
                start();
        }

        // 包络：衰减到 floor 后保持；休止条目内上一个音线性淡出
        amplitude_ = 0;
        if (index_ < count_) {
            decay_ *= decay_step_;
            float amp = floor_ + (1 - floor_) * decay_;
            if (notes_[index_].midi == 0) {
                fade_ = std::max(fade_ - fade_step_, 0.0f);
                amp *= fade_;
            }
            amplitude_ = weight_ * amp;
        }

        // 波形，相位跨音符连续
        phase_ += phase_step_;
        if (phase_ >= 1)
            phase_ -= 1;
        float x = 0;
        if (wave_ == Wave::SQUARE)
            x = square_blep(phase_, phase_step_);
        else if (wave_ == Wave::TRIANGLE)
            x = 1 - 4 * std::fabs(phase_ - 0.5f); // 三角波谐波按 1/n² 衰减，80 kHz 下混叠可忽略
        else {
            // 亮表随包络衰减过渡到暗表，模拟高次谐波衰减更快；共振峰同样随衰减减弱到一半，和的峰值不超过 1
            float h = phase_ * static_cast<float>(harmonic_);
            h -= static_cast<float>(static_cast<int>(h));
            const float boost = formant_ * (0.5f + 0.5f * decay_);
            x = (piano_wave(phase_, decay_) + boost * sine_wave(h)) / (1 + formant_);
        }

        // 低通作用在加包络后的输出，状态跨音符连续，换音不出爆音
        output_ += lowpass_ * (amplitude_ * x - output_);
        return output_;
    }

    // 本采样包络幅度，范围 [0, weight]
    float amplitude() const {
        return amplitude_;
    }

    bool is_done() const {
        return index_ >= count_;
    }

  private:
    static constexpr float UNIT = 0.01f;    // 曲目时长单位 s
    static constexpr float RELEASE = 0.03f; // 松开后淡出时间上限 s

    const VoiceNote* const notes_;
    const size_t count_;
    const float weight_, tau_a4_, floor_;
    const Wave wave_;
    const float lowpass_; // 一阶低通系数，1 为直通
    const float formant_, formant_hz_;

    size_t index_ = 0;
    float left_ = 0;                   // 当前条目剩余时间 s
    float phase_ = 0, phase_step_ = 0; // 相位 [0, 1) 与每采样增量；增量为 0 表示尚未发声
    float decay_ = 0, decay_step_ = 1; // exp(-age/tau) 与每采样衰减系数
    float fade_ = 0, fade_step_ = 0;   // 休止内淡出系数与每采样减量
    float amplitude_ = 0;
    float output_ = 0; // 低通状态
    int harmonic_ = 1; // 共振峰增强的谐波次数

    // 进入 index_ 条目
    void start() {
        const VoiceNote& note = notes_[index_];
        const float len = static_cast<float>(note.len) * UNIT;
        left_ += len;

        // 休止：上一个音在 RELEASE 内淡出，不超过休止时长；曲首休止无音可淡出
        if (note.midi == 0) {
            fade_ = phase_step_ > 0 ? 1.0f : 0.0f;
            fade_step_ = SAMPLE_DT / std::min(RELEASE, len);
            return;
        }

        // 起音：低音衰减慢、高音衰减快
        const float hz = 440.0f * std::exp2(static_cast<float>(note.midi - 69) / 12.0f);
        const float tau = std::clamp(tau_a4_ * std::sqrt(440.0f / hz), 0.3f, 2.0f);
        phase_step_ = hz * SAMPLE_DT;
        harmonic_ = std::max(1, static_cast<int>(std::lround(formant_hz_ / hz)));
        decay_ = 1;
        decay_step_ = std::exp(-SAMPLE_DT / tau);
    }
};
