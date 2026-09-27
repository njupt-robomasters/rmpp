#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/pwm.hpp"
#include "bsp/storage.hpp"
#include "bsp/uart.hpp"
#include "mavlink/mavlink.hpp"
#include "misc/buzzer.hpp"
#include "misc/rgb.hpp"
#include "rc/rc.hpp"
#include "referee/referee.hpp"
#include "ui/ui.hpp"

inline Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
inline Buzzer buzzer(Pwm(4, 3));
inline Storage storage("hero");
inline FSi6X fsi6x({.uart = Uart(3)});
inline VT13 vt13({.uart = Uart(6)});
inline Rc rc({
                 .vxy_max = 5.0f,    // 100W下2.4m/s，不限功率4.5m/s
                 .wr_max = 90 * rpm, // 100W下65rpm，不限功率90rpm
                 .axy = 100.0f,
                 .dxy = 100.0f,
                 .ar = 1000 * rpm,
                 .dr = 1000 * rpm,

                 .wyaw = 360 * deg_s,
                 .wpitch = 360 * deg_s,
                 .ayaw = 3600 * deg_s,
                 .apitch = 3600 * deg_s,
             },
             fsi6x, vt13);
inline Mavlink mavlink({.cdc = Cdc(1)});
inline Referee referee({.can = Can(1)});
inline Ui ui({.can = Can(1), .max_bullet = 20.0f});
