#pragma once

// 板载无源蜂鸣器
class Buzzer {
  public:
    // 以 50% 占空比按 hz 发声，hz 须大于 0
    void set_tone(float hz) const;

    void mute() const;
};
