#pragma once

#include "can.hpp"
#include "cdc.hpp"
#include "led.hpp"
#include "timer.hpp"
#include "uart.hpp"

namespace rmpp {
class Board {
  public:
    Can can(int id) const;
    Uart uart(int id) const;
    Cdc cdc(int id) const;
    Led led(int id) const;
    Pwm pwm(int timer, int channel) const;
    Timer timer() const { return Timer{}; }
};
inline constexpr Board board{};
} // namespace rmpp
