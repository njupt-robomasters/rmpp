#pragma once

class Pwm {
  public:
    Pwm(int timer, int channel);

    // 同一定时器各通道共享频率，hz 须大于 0
    void set_frequency(float hz) const;
    void set_duty(float duty) const;

    struct Context;
    Context* context_ = nullptr;
};
