#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/pwm.hpp"
#include "bsp/storage.hpp"
#include "bsp/uart.hpp"
#include "minipc/aim.hpp"
#include "misc/buzzer.hpp"
#include "misc/rgb.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "referee/ui.hpp"

inline Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
inline Buzzer buzzer(Pwm(4, 3));
inline Storage storage("hero");
inline VT13 vt13({.uart = Uart(6)});
inline FSi6X fsi6x({.uart = Uart(3)});
inline Aim aim({.cdc = Cdc(1)});
inline Referee referee({.bus = Can(1)});
inline RobotUi ui(referee, {.max_bullet = 20.0f});

void misc_loop();
