#pragma once

#include "bsp/can.hpp"
#include "bsp/pwm.hpp"
#include "bsp/spi.hpp"

#include "imu/imu.hpp"
#include "math/unit.hpp"
#include "motor/dji/gm6020.hpp"
#include "motor/dm/dm4310.hpp"
#include "motor/lk/mf9025.hpp"
#include "robot/gimbal/gimbal_dual_yaw.hpp"

// 惯性测量单元
inline Imu::Dir imu_dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline Imu::Calib imu_calib = {.gx_offset = -0.000674135052, .gy_offset = 0.00137436262, .gz_offset = 0.00146490871, .g_norm = 9.54711819};
inline Imu imu(imu_dir, imu_calib, {.accel = Spi(1, 1), .gyro = Spi(1, 2), .heater = Pwm(10, 1)});

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
    .protocol = {.can = Can(1), .master_id = 0x141, .slave_id = 0x141},
    .install = {.is_invert = false},
    .control = {.control_mode = Motor::SINGLE_TURN_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .position_pid_config = &yaw1_pid},
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
    .protocol = {.can = Can(1), .master_id = 0x205, .slave_id = 0x1FF},
    .install = {.offset = 0 * deg},
    .control = {.control_mode = Motor::SINGLE_TURN_SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &yaw2_speed_pid, .position_pid_config = &yaw2_angle_pid, .is_limit = true, .position_min = -40 * deg, .position_max = 40 * deg},
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
    .install = {.offset = -72 * deg, .is_invert = true},
    .control = {.control_mode = Motor::SINGLE_TURN_SPEED_MODE, .pid_output_type = Motor::TORQUE_OUTPUT, .speed_pid_config = &pitch_speed_pid, .position_pid_config = &pitch_angle_pid, .is_limit = true, .position_min = -22 * deg, .position_max = 15 * deg},
});

// 云台
inline GimbalDualYaw gimbal(imu, {yaw1, yaw2, pitch});

void gimbal_loop();
