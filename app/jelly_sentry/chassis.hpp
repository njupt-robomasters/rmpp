#pragma once

#include "motor/m3508.hpp"
#include "power/super_cap.hpp"
#include "robot/chassis_matrix.hpp"

// 导航满速平移时的小陀螺角速度，rad/s；与底盘功率控制保留转速一致
static constexpr float WR_MIN = 60 * rpm;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 312.5f * mm;
static constexpr float WHEEL_RADIUS = 66.0f * mm;
static constexpr float MAX_F = M3508::MAX_TORQUE / WHEEL_RADIUS * 4; // 四轮满电流时的总牵引力，N

// 底盘速度 PID：速度误差 m/s → 牵引力 N
inline Pid::Config vxyz_pid_config = {
    .kp = MAX_F / 1.0f,
    .ki = MAX_F / 0.01f,
    .max_i = MAX_F,
    .max_out = MAX_F,
    .err_fc = 10.0f,
};

// 底盘跟随 PID：云台相对底盘角度 rad → 旋转角速度 rad/s
inline Pid::Config follow_pid_config = {
    .kp = 5.0f,
    .max_out = 360.0f * deg,
};

// 底盘电机（左前、左后、右后、右前）
inline M3508 w1({
    .can = Can(1),
    .rx_id = 0x201,
    .is_invert = false,
});
inline M3508 w2({
    .can = Can(1),
    .rx_id = 0x202,
    .is_invert = false,
});
inline M3508 w3({
    .can = Can(1),
    .rx_id = 0x203,
    .is_invert = false,
});
inline M3508 w4({
    .can = Can(1),
    .rx_id = 0x204,
    .is_invert = false,
});

// 超级电容
inline SuperCap super_cap({.can = Can(1)});

// 底盘
// 力控：底盘速度 PID 输出牵引力，力学逆解为各轮电流
inline ChassisMatrixTorque chassis({&w1, &w2, &w3, &w4}, {
                                                             .kinematics = ChassisMatrix::OMNI,
                                                             .chassis_radius = CHASSIS_RADIUS,
                                                             .wheel_radius = WHEEL_RADIUS,
                                                             .wheel_resistance = 0.194f,
                                                             .speed_pid_config = &vxyz_pid_config,
                                                             .follow_pid_config = &follow_pid_config,
                                                         });

void chassis_loop();
