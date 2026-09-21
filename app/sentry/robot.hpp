#pragma once

#include "robot/sentry.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/conversions.hpp"
#include "misc.hpp"
#include "shooter.hpp"

using math::deg_s;
using math::rpm;

inline Sentry::Config robot_config = {
    .vxy_max = 5.0f,     // 100W下2.3m/s，不限功率3.2m/s
    .wr_max = 120 * rpm, // 100W下107rpm，不限功率155rpm
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
inline Sentry robot(robot_config,
                    {led,
                     buzzer,
                     flashdb,

                     rc,
                     mavlink,

                     chassis,
                     gimbal,
                     shooter,

                     referee,
                     ui});