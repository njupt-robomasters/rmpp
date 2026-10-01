#pragma once

#include "bsp/can.hpp"
#include "bsp/pwm.hpp"
#include "bsp/spi.hpp"

#include "motor/dji/gm6020.hpp"
#include "motor/dm/dm4310.hpp"
#include "robot/gimbal/gimbal_classic.hpp"

// 惯性测量单元
inline Imu imu({
    .hardware = {.accel = Spi(1, 1), .gyro = Spi(1, 2), .heater = Pwm(10, 1)},
    .adjust = {.gyro_offset = {0.00316276052f, -0.00411193585f, -0.00242328411f}, .gravity = 9.79961872f},
});

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
    .bus = {.can = Can(1), .rx_id = 0x205},
    .install = {.is_invert = false},
    .control = {.mode = Motor::SINGLE_TURN_POSITION_SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &yaw_speed_pid, .position_pid_config = &yaw_angle_pid},
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
    .motor = {
        .bus = {.can = Can(2), .tx_id = 0x11, .rx_id = 0x10},
        .install = {.offset = -150 * deg, .is_invert = false},
        .control = {.mode = Motor::SINGLE_TURN_POSITION_SPEED_MODE, .pid_output_type = Motor::TORQUE_OUTPUT, .speed_pid_config = &pitch_speed_pid, .position_pid_config = &pitch_angle_pid, .is_limit = true, .position_min = -15 * deg, .position_max = 30 * deg},
    },
});

// 云台
inline GimbalClassic gimbal(imu, {yaw, pitch});
