#pragma once

#include "robot/robot.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/unit.hpp"
#include "misc.hpp"
#include "shooter.hpp"

inline Robot::Config robot_config = {
    .vxy_max = 5.0f,     // 75W下2.4m/s，不限功率6.3m/s
    .wr_max = 150 * rpm, // 75W下125rpm，不限功率240rpm
    .axy = 100.0f,
    .dxy = 100.0f,
    .ar = 1000 * rpm,
    .dr = 1000 * rpm,

    .wyaw = 360 * deg_s,
    .wpitch = 360 * deg_s,
    .ayaw = 3600 * deg_s,
    .apitch = 3600 * deg_s,

    .bullet_speed = 20.0f,
    .bullet_freq = 20,
    .heat_protect = 80,
};
inline Robot robot(robot_config,
                   {rgb,
                    buzzer,
                    storage,

                    rc,
                    mavlink,

                    chassis,
                    gimbal,
                    shooter,

                    referee,
                    ui});