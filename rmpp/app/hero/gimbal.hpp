#pragma once

#include "motor/GM6020.hpp"
#include "motor/DM6006.hpp"
#include "module/gimbal/Gimbal_Classic.hpp"
#include "imu.hpp"

// yaw电机
inline PID::config_t yaw_speed_pid = {
    .kp = (3 * A) / (100 * deg_s),
    .ki = (3 * A) / (60 * deg),
    .max_i = 3 * A,
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t yaw_angle_pid = {
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .is_invert = true,
    .offset = -32.5f * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw_speed_pid,
    .angle_pid_config = &yaw_angle_pid,
});

// pitch电机
inline PID::config_t pitch_speed_pid = {
    .kp = (4 * Nm) / (150 * deg_s),
    .max_out = 4 * Nm,
    .fc = 10 * Hz,
};
inline PID::config_t pitch_angle_pid = {
    .kp = 20 * default_unit,
    .max_out = 720 * deg_s,
};
inline DM6006 pitch({
    .can_port = 2,
    .master_id = 0x12,
    .slave_id = 0x13,
    .is_invert = false,
    .offset = 172.5f * deg,
    .is_limit = true,
    .limit_min = -10 * deg,
    .limit_max = 15 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
});

// 云台
inline Gimbal_Classic gimbal(imu, {yaw, pitch});