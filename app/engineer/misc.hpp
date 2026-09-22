#pragma once

#include "bsp/bsp.hpp"

#include "bsp/storage.hpp"
#include "mavlink/mavlink.hpp"
#include "misc/buzzer.hpp"
#include "rc/rc.hpp"
#include "referee/referee.hpp"
#include "rgb/rgb.hpp"
#include "ui/ui.hpp"

inline Rgb rgb({.red = bsp.pwm(5, 3), .green = bsp.pwm(5, 2), .blue = bsp.pwm(5, 1)});
inline Buzzer buzzer(bsp.pwm(4, 3));
inline Storage storage("engineer");
inline Rc rc({.uart = bsp.uart(3)}, {.uart = bsp.uart(6)});
inline Mavlink mavlink;
inline Referee referee({.can = bsp.can(1)});
inline Ui ui({
    .can = bsp.can(1),
});