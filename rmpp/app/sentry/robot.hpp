#pragma once

#include "module/robot/Sentry.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Sentry::config_t robot_config = {
    .vxy_max = 2 * m_s, // 极限3.2m/s
    .wr_max = 60 * rpm, // 极限155rpm
    .axy = 10 * m_ss,
    .dxy = 10 * m_ss,

    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,

    .bullet_speed = 20 * m_s,
    .bullet_freq = 5 * Hz,
    .heat_protect = 0,
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