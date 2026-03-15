#pragma once

#include "module/robot/Sentry.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Sentry::config_t robot_config = {
    .vxy_max = 3 * m_s,  // 极限3.2m/s
    .wr_max = 150 * rpm, // 极限155rpm
    .axy = 500 * m_ss,
    .dxy = 500 * m_ss,
    .ar = 12000 * rpm,
    .dr = 12000 * rpm,

    .wyaw = 360 * deg_s,
    .wpitch = 360 * deg_s,
    .ayaw = 36000 * deg_s,
    .apitch = 36000 * deg_s,

    .bullet_speed = 20 * m_s,
    .bullet_freq = 20 * Hz,
    .heat_protect = 80,
};
inline Sentry robot(robot_config,
                    {
                        led,
                        buzzer,
                        flashdb,

                        rc,
                        mavlink,

                        chassis,
                        gimbal,
                        shooter,

                        referee,
                        ui
                    });