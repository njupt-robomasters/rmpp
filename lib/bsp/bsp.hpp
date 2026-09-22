#pragma once

#include "can.hpp"
#include "cdc.hpp"
#include "pwm.hpp"
#include "spi.hpp"
#include "timer.hpp"
#include "uart.hpp"

class Bsp {
  public:
    Can can(int id) const {
        return Can(id);
    }

    Uart uart(int id) const {
        return Uart(id);
    }

    Cdc cdc(int id) const {
        return Cdc(id);
    }

    Pwm pwm(int timer, int channel) const {
        return Pwm(timer, channel);
    }

    Spi spi(int bus, int chip_select) const {
        return Spi(bus, chip_select);
    }

    Timer timer() const {
        return Timer{};
    }
};

inline constexpr Bsp bsp{};
