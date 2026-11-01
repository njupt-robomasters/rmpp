#include "melody.hpp"
#include <cmath>

void Melody::play(const std::span<const Note> music) {
    music_ = music;
    idx_ = 0;
}

void Melody::stop() {
    music_ = {};
    idx_ = 0;
    buzzer_.mute();
}

void Melody::update() {
    // 未开始播放
    if (music_.empty())
        return;

    // 逐音符计时，首个音符立即开始
    if (idx_ > 0 && timer_.elapsed() < static_cast<float>(music_[idx_ - 1].len) * 0.01f)
        return;

    // 播放完成
    if (idx_ >= music_.size()) {
        stop();
        return;
    }

    // 只在音高切换时操作蜂鸣器
    const auto midi = music_[idx_].midi;
    if (idx_ == 0 || midi != music_[idx_ - 1].midi) {
        if (midi == 0)
            buzzer_.mute();
        else
            buzzer_.set_tone(440.0f * std::exp2((static_cast<float>(midi) - 69.0f) / 12.0f));
    }

    idx_++;
    timer_.restart();
}
