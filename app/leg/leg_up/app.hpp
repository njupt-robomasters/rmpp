#pragma once

#include "math/unit.hpp"
#include "melody/melody.hpp"
#include "rc/vt13.hpp"
#include <cmath>

inline Melody melody({});

// 遥控器：键盘加减速度待实车整定
inline VT13 vt13({
    .uart = Uart(6),
    .vxy_max = 3.0f,
    .axy = INFINITY,
    .dxy = INFINITY,
    .wr_max = 360 * deg_s,
    .ar = INFINITY * deg_ss,
    .dr = INFINITY * deg_ss,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 25.0f,
    .bullet_freq = 10.0f,
});
