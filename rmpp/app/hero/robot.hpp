#pragma once

#include "robot/Robot.hpp"

#include "misc.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

inline Robot::config_t robot_config = {
    .vxy_max = 5 * m_s, // 100W下2.4m/s，不限功率4.5m/s
    .wr_max = 90 * rpm, // 100W下65rpm，不限功率90rpm
    .axy = 100 * m_ss,
    .dxy = 100 * m_ss,
    .ar = 1000 * rpm,
    .dr = 1000 * rpm,

    .wyaw = 360 * deg_s,
    .wpitch = 360 * deg_s,
    .ayaw = 3600 * deg_s,
    .apitch = 3600 * deg_s,

    .bullet_speed = 15.0f * m_s,
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