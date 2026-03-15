#pragma once

#include "module/robot/Robot.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Robot::config_t robot_config = {
    .vxy_max = 6 * m_s,  // 极限6.3m/s
    .wr_max = 240 * rpm, // 极限240rpm
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
inline Robot robot(robot_config,
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