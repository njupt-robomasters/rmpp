#pragma once

#include "module/robot/Infantry.hpp"

#include "rc.hpp"
#include "mavlink.hpp"

#include "imu.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

#include "referee.hpp"
#include "ui.hpp"

inline Infantry::config_t robot_config = {
    .vxy_max = 1 * m_s, // 极限4.5m/s
    .wr_max = 60 * rpm, // 极限130rpm
    .axy = 10 * m_ss,
    .dxy = 10 * m_ss,

    .yaw_max = 360 * deg_s,
    .pitch_max = 360 * deg_s,

    .bullet_speed = 12 * m_s,
    .bullet_freq = 1 * Hz,
    .heat_protect = 0
};
inline Infantry robot(robot_config,
                      {
                          rc,
                          mavlink,

                          imu,

                          chassis,
                          gimbal,
                          shooter,

                          referee,
                          ui
                      });