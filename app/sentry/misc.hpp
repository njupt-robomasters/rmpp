#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/rgb.hpp"
#include "bsp/uart.hpp"

#include "bsp/kv_store.hpp"
#include "math/unit.hpp"
#include "melody/melody.hpp"
#include "minipc/aim.hpp"
#include "minipc/nav.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "referee/robot_ui.hpp"

inline Rgb rgb;
inline Melody melody({});
inline KvStore kv_store("sentry");

// 遥控器：平移 100W 下 2.3m/s、不限功率 3.2m/s，旋转 100W 下 107rpm、不限功率 155rpm；最高弹频由 C、Z 键调节
inline VT13 vt13({
    .uart = Uart(6),
    .vxy_max = 5.0f,
    .axy = 100.0f,
    .dxy = 100.0f,
    .wr_max = 120 * rpm,
    .ar = 1000 * rpm,
    .dr = 1000 * rpm,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 20.0f,
    .bullet_freq = 20.0f,
});

inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(1)});
inline Referee referee({.can = Can(1)});
inline RefereeSender referee_sender({.can = Can(1)});
inline RobotUi ui({});

void misc_loop();
