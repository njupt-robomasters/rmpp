#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/rgb.hpp"
#include "bsp/uart.hpp"

#include "bsp/storage.hpp"
#include "minipc/aim.hpp"
#include "misc/melody.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "referee/ui.hpp"

inline Rgb rgb;
inline Melody melody;
inline Storage storage("infantry");
inline VT13 vt13({.uart = Uart(6)});
inline FSi6X fsi6x({.uart = Uart(3)});
inline Aim aim({.cdc = Cdc(1)});
inline Referee referee({.bus = Can(1)});
inline RobotUi ui(referee);

void misc_loop();
