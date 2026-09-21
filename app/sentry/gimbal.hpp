#pragma once

#include "imu.hpp"
#include "math/conversions.hpp"
#include "module/gimbal/gimbal_dual_yaw.hpp"
#include "motor/dm4310.hpp"
#include "motor/gm6020.hpp"
#include "motor/mf9025.hpp"

using math::deg;
using math::deg_s;
using math::deg_ss;

// 用于Ozone调参
// #define yaw1_pid angle_pid
// #define yaw1 motor
//
// #define yaw2_speed_pid speed_pid
// #define yaw2_angle_pid angle_pid
// #define yaw2 motor
//
// #define pitch_speed_pid speed_pid
// #define pitch_angle_pid angle_pid
// #define pitch motor

// 大yaw电机
inline Pid::Config yaw1_pid = {
    .kp = (9) / (1.8 * deg),
    .kd = (9) / (18 * deg_s),
    .max_out = 9,
    .fc = 10,
};
inline MF9025 yaw1({
    .can_port = 1,
    .master_id = 0x141,
    .slave_id = 0x141,
    .is_invert = false,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &yaw1_pid,
});

// 小yaw电机
inline Pid::Config yaw2_speed_pid = {
    .kp = (3) / (300 * deg_s),
    .kd = (3) / (30000 * deg_ss),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config yaw2_angle_pid = {
    .kp = 30.0f,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw2({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .offset = 0 * deg,
    .is_limit = true,
    .limit_min = -40 * deg,
    .limit_max = 40 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw2_speed_pid,
    .angle_pid_config = &yaw2_angle_pid,
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
    .can_port = 2,
    .master_id = 0x10,
    .slave_id = 0x11,
    .is_invert = true,
    .offset = -72 * deg,
    .is_limit = true,
    .limit_min = -22 * deg,
    .limit_max = 15 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
});

// 云台
inline GimbalDualYaw gimbal(imu, {yaw1, yaw2, pitch});