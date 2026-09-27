#pragma once

#include "bsp/can.hpp"
#include "bsp/pwm.hpp"
#include "bsp/spi.hpp"

#include "motor/dji/gm6020.hpp"
#include "motor/dm/dm4310.hpp"
#include "robot/gimbal/gimbal_classic.hpp"

// 惯性测量单元
inline Imu::Dir imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline Imu::Calib imu_calib = {.gx_offset = 0.00316276052, .gy_offset = -0.00411193585, .gz_offset = -0.00242328411, .g_norm = 9.79961872};
inline Imu imu(imu_dir, imu_calib, {.accel = Spi(1, 1), .gyro = Spi(1, 2), .heater = Pwm(10, 1)});

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
    .protocol = {.can = Can(1), .master_id = 0x205, .slave_id = 0x1FF},
    .install = {.is_invert = false},
    .control = {.control_mode = Motor::SINGLE_TURN_SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &yaw_speed_pid, .position_pid_config = &yaw_angle_pid},
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
    .protocol = {.can = Can(2), .master_id = 0x10, .slave_id = 0x11},
    .install = {.offset = -150 * deg, .is_invert = false},
    .control = {.control_mode = Motor::SINGLE_TURN_SPEED_MODE, .pid_output_type = Motor::TORQUE_OUTPUT, .speed_pid_config = &pitch_speed_pid, .position_pid_config = &pitch_angle_pid, .is_limit = true, .position_min = -15 * deg, .position_max = 30 * deg},
});

// 云台
inline GimbalClassic gimbal(imu, {yaw, pitch});
