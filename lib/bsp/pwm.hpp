#pragma once

class Pwm {
  public:
    Pwm() = default;
    Pwm(int timer, int channel);

    explicit operator bool() const {
        return state_ != nullptr;
    }

    bool set_frequency(float hz) const;
    bool set_duty(float duty) const;

  private:
    struct State;
    int timer_ = 0, channel_ = 0;
    State* state_ = nullptr;
    friend void pwm_init();
};
