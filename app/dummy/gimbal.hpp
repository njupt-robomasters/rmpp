#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/rmdl7025.hpp"

// 惯性测量单元与姿态解算，C 板装在底盘上，结合 yaw 电机角度换算云台朝向；静止漂移明显时重新校准
static Imu imu({.orientation = euler_to_quaternion({.yaw = 0 * deg, .pitch = 0 * deg, .roll = 180 * deg}),
                .gyro_offset = Vector3f(0.00153f, -0.00384f, -0.00022f),
                .gravity = 9.71541f});
inline QuaternionEkf attitude(imu, {});

// 云台位置 P 控制，世界系朝向误差 rad → 电流 A；待实车整定
inline Pid::Config yaw_pid_config = {
    .kp = 2.0f,
    .kd = 0.1f,
    .max_out = RMDL7025::MAX_CURRENT,
    .d_fc = 30.0f,
};

// 云台电机：RMD-L-7025 直连 CAN1，电机 ID 1；position 为云台相对底盘前方的角度，逆时针为正，零点待标定
inline RMDL7025 yaw({
    .can = Can(2),
    .tx_id = 0x141,
    .rx_id = 0x241,
    .is_invert = false,
    .offset = 2.97f,
});

void gimbal_loop();
