#pragma once

#include <cstdint>
#include <span>

#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"

// MIDI 音符，midi 为 0 表示休止；len 单位 10 ms，曲目由 tools/midi_to_music.py 生成，见 docs/music.md
struct Note {
    uint8_t midi;
    uint8_t len;
};

class Buzzer {
  public:
    explicit Buzzer(const Pwm pwm)
        : pwm_(pwm) {}

    // 从头播放曲目，打断当前播放；曲目须在播放期间保持有效
    void play(std::span<const Note> music);

    // 停止播放并重置状态
    void stop();

    // 周期调用，到时切换音符
    void update();

  private:
    // 输出
    Pwm pwm_;

    // 播放进度，idx_ 指向下一个音符，timer_ 对当前音符计时
    std::span<const Note> music_;
    size_t idx_ = 0;
    Timer timer_;
};
