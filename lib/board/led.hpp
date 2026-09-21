#pragma once

#include "pwm.hpp"

namespace rmpp {
class Led {
  public:
    Led() = default;
    explicit operator bool() const { return bool(red_); }
    bool set_rgb(float red, float green, float blue) const;
    bool set_hsv(float hue, float saturation, float brightness) const;

  private:
    Pwm red_, green_, blue_;
    Led(Pwm red, Pwm green, Pwm blue) : red_(red), green_(green), blue_(blue) {}
    friend class Board;
};
} // namespace rmpp
