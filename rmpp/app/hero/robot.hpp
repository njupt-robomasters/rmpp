#pragma once

#include "module/robot/Infantry.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Infantry::config_t robot_config = {
    .vxy_max = 5 * m_s,  // 极限4.5m/s
    .wr_max = 120 * rpm, // 极限130rpm
    .axy = 10 * m_ss,
    .dxy = 10 * m_ss,

    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,

    .bullet_speed = 15 * m_s,
    .bullet_freq = 1 * Hz,
    .heat_protect = 0
};
inline Infantry robot(robot_config,
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