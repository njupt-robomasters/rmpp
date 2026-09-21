#pragma once

namespace rmpp {
class Pwm {
  public:
    Pwm() = default;
    explicit operator bool() const { return state_ != nullptr; }
    bool set_frequency(float hz) const;
    bool set_duty(float duty) const;

  private:
    struct State;
    State* state_ = nullptr;
    explicit Pwm(State* state) : state_(state) {}
    static State* find(int timer, int channel);
    friend class Board;
    friend void pwm_init();
};
} // namespace rmpp
