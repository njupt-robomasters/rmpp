#pragma once

#include "bsp/bsp.hpp"

#include "imu.hpp"
#include "math/unit.hpp"
#include "module/gimbal/gimbal_classic.hpp"
#include "motor/dm4310.hpp"
#include "motor/gm6020.hpp"

// 用于Ozone调参
// #define yaw_speed_pid speed_pid
// #define yaw_angle_pid angle_pid
// #define yaw motor
//
// #define pitch_speed_pid speed_pid
// #define pitch_angle_pid angle_pid
// #define pitch motor

// yaw电机
inline Pid::Config yaw_speed_pid = {
    .kp = (3) / (120 * deg_s),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config yaw_angle_pid = {
    .kp = 20.0f,
    .kd = 0.2,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw({
    .can = bsp.can(1),
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .is_invert = false,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw_speed_pid,
    .angle_pid_config = &yaw_angle_pid,
});

// pitch电机
inline Pid::Config pitch_speed_pid = {
    .kp = (3) / (150 * deg_s),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config pitch_angle_pid = {
    .kp = 20.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch({
    .can = bsp.can(2),
    .master_id = 0x10,
    .slave_id = 0x11,
    .is_invert = false,
    .offset = -150 * deg,
    .is_limit = true,
    .limit_min = -15 * deg,
    .limit_max = 30 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
});

// 云台
inline GimbalClassic gimbal(imu, {yaw, pitch});