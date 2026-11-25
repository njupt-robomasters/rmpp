#pragma once

#include "melody/melody.hpp"
#include "rc/vt13.hpp"
#include <cmath>

inline Melody melody({});

// 遥控器
inline VT13 vt13({
    .uart = Uart(6),
    .vxy_max = 3.0f,
    .axy = INFINITY,
    .dxy = INFINITY,
    .wr_max = 120 * rpm,
    .ar = INFINITY,
    .dr = INFINITY,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 20.0f,
    .bullet_freq = 10.0f,
});
