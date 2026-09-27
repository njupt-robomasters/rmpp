#pragma once

class Pwm {
  public:
    Pwm(int timer, int channel);

    explicit operator bool() const {
        return context_ != nullptr;
    }

    bool set_frequency(float hz) const;
    bool set_duty(float duty) const;

    struct Context;
    Context* context_ = nullptr;
};
