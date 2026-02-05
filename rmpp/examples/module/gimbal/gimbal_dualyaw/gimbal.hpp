#pragma once

#include "motor/MF9025.hpp"
#include "motor/GM6020.hpp"
#include "motor/DM4310.hpp"
#include "module/gimbal/Gimbal_DualYaw.hpp"
#include "imu.hpp"

// 大yaw电机
inline PID::config_t yaw1_pid = {
    .kp = (16.5 * A) / (16.5 * deg),
    .kd = (16.5 * A) / (165 * deg_s),
    .max_out = 16.5 * A,
    .fc = 10 * Hz,
};
inline MF9025 yaw1({
    .can_port = 1,
    .master_id = 0x141,
    .slave_id = 0x141,
    .is_invert = false,
    .offset = -115 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &yaw1_pid,
});

// 小yaw电机
inline PID::config_t yaw2_speed_pid = {
    .kp = (3 * A) / (100 * deg_s),
    .ki = (3 * A) / (60 * deg),
    .max_i = 3 * A,
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t yaw2_angle_pid = {
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw2({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .offset = -120 * deg,
    .is_limit = true,
    .limit_min = -90 * deg,
    .limit_max = 90 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw2_speed_pid,
    .angle_pid_config = &yaw2_angle_pid,
});

// pitch电机
inline PID::config_t pitch_speed_pid = {
    .kp = (3 * Nm) / (100 * deg_s),
    .ki = (3 * Nm) / (60 * deg),
    .max_i = 3 * Nm,
    .max_out = 3 * Nm,
    .fc = 10 * Hz,
};
inline PID::config_t pitch_angle_pid = {
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch({
    .can_port = 2,
    .master_id = 0x10,
    .slave_id = 0x11,
    .is_invert = true,
    .offset = -162 * deg,
    .is_limit = true,
    .limit_min = -35 * deg,
    .limit_max = 40 * deg,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
});

// 云台
inline Gimbal_DualYaw gimbal(imu, {yaw1, yaw2, pitch});