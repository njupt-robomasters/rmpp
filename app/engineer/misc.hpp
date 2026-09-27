#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/pwm.hpp"
#include "bsp/uart.hpp"

#include "bsp/storage.hpp"
#include "mavlink/mavlink.hpp"
#include "misc/buzzer.hpp"
#include "misc/rgb.hpp"
#include "rc/rc.hpp"
#include "referee/referee.hpp"
#include "ui/ui.hpp"

inline Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
inline Buzzer buzzer(Pwm(4, 3));
inline Storage storage("engineer");
inline FSi6X fsi6x({.uart = Uart(3)});
inline VT13 vt13({.uart = Uart(6)});
inline Rc rc({}, fsi6x, vt13);
inline Mavlink mavlink({.cdc = Cdc(1)});
inline Referee referee({.can = Can(1)});
inline Ui ui({
    .can = Can(1),
});
