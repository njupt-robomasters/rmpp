#pragma once

#include "robot/Robot.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Robot::config_t robot_config = {
    .vxy_max = 4 * m_s,  // 极限4.5m/s
    .wr_max = 120 * rpm, // 极限130rpm
    .axy = 500 * m_ss,
    .dxy = 500 * m_ss,
    .ar = 12000 * rpm,
    .dr = 12000 * rpm,

    .wyaw = 360 * deg_s,
    .wpitch = 360 * deg_s,
    .ayaw = 36000 * deg_s,
    .apitch = 36000 * deg_s,

    .bullet_speed = 15 * m_s,
    .heat_protect = 100,
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