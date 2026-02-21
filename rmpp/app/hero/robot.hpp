#pragma once

#include "module/robot/Infantry.hpp"

#include "fsi6x.hpp"
#include "vt13.hpp"
#include "mavlink.hpp"

#include "imu.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

#include "referee.hpp"
#include "ui.hpp"

inline Infantry::config_t robot_config = {
    .vxy_max = 1 * m_s,
    .wr_max = 60 * rpm,
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
                          fsi6x,
                          vt13,
                          mavlink,

                          imu,

                          chassis,
                          gimbal,
                          shooter,

                          referee,
                          ui
                      });