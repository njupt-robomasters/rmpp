#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/pwm.hpp"
#include "bsp/uart.hpp"

#include "bsp/storage.hpp"
#include "mavlink/mavlink.hpp"
#include "misc/buzzer.hpp"
#include "misc/rgb.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "ui/ui.hpp"

inline Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
inline Buzzer buzzer(Pwm(4, 3));
inline Storage storage("sentry");
inline VT13 vt13({.uart = Uart(6)});
inline FSi6X fsi6x({.uart = Uart(3)});
inline Mavlink mavlink({.cdc = Cdc(1), .send_freq = 150});
inline Referee referee({.can = Can(1)});
inline Ui ui({
    .can = Can(1),
});

void misc_loop();
