#pragma once

#include "module/robot/Sentry.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Sentry::config_t robot_config = {
    .vxy_max = 3 * m_s,  // 极限3.2m/s
    .wr_max = 150 * rpm, // 极限155rpm
    .axy = 100 * m_ss,
    .dxy = 100 * m_ss,
    .ar = 1000 * rpm,
    .dr = 1000 * rpm,

    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,

    .bullet_speed = 20 * m_s,
    .bullet_freq = 20 * Hz,
    .heat_protect = 0
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