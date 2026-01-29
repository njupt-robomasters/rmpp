#pragma once

#include "motor/GM6020.hpp"
#include "motor/DM4310.hpp"
#include "module/gimbal/Gimbal_Classic.hpp"
#include "imu.hpp"

// yaw电机
inline PID::config_t yaw_speed_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (3 * A) / (100 * deg_s),
    .ki = (3 * A) / (60 * deg),
    .max_i = 3 * A,
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t yaw_angle_pid = {
    .mode = PID::POSITION_MODE,
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .reduction = GM6020::reduction,
    .Kt = GM6020::Kt,
    .R = GM6020::R,
    .is_invert = false,
    .offset = 96.72f * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw_speed_pid,
    .angle_pid_config = &yaw_angle_pid,
});

// pitch电机
inline PID::config_t pitch_speed_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (3 * Nm) / (100 * deg_s),
    .ki = (3 * Nm) / (60 * deg),
    .max_i = 3 * Nm,
    .max_out = 3 * Nm,
    .fc = 10 * Hz,
};
inline PID::config_t pitch_angle_pid = {
    .mode = PID::POSITION_MODE,
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch({
    .can_port = 2,
    .master_id = 0x10,
    .slave_id = 0x11,
    .reduction = DM4310::reduction,
    .Kt = DM4310::Kt,
    .R = DM4310::R,
    .is_invert = false,
    .offset = -150 * deg,
    .is_limit = true,
    .limit_min = -20 * deg,
    .limit_max = 25 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
});

// 云台
inline Gimbal_Classic gimbal(imu, {yaw, pitch});