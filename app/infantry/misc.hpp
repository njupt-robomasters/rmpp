#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/rgb.hpp"
#include "bsp/uart.hpp"

#include "bsp/kv_store.hpp"
#include "minipc/aim.hpp"
#include "melody/melody.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "referee/ui.hpp"

inline Rgb rgb;
inline Melody melody({});
inline KvStore kv_store("infantry");
inline VT13 vt13({.uart = Uart(6)});
inline FSi6X fsi6x({.uart = Uart(3)});
inline Aim aim({.cdc = Cdc(1)});
inline Referee referee({.bus = Can(1)});
inline RobotUi ui(referee);

void misc_loop();
