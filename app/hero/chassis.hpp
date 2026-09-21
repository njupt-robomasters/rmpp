#pragma once

#include "math/conversions.hpp"
#include "module/chassis/chassis_mecanum.hpp"
#include "motor/m3508.hpp"

using math::cm;
using math::deg;

// 底盘电机参数
static constexpr float REDUCTION = 268.0f / 17.0f;
static constexpr float KT = M3508::KT / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_INVERT = false;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 25.87f * cm;
static constexpr float WHEEL_RADIUS = 7.1f * cm;
static constexpr float MAX_F = M3508::MAX_CURRENT * KT / WHEEL_RADIUS * 4;

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10,
};
inline Pid::Config follow_pid = {
    .kp = 5.0f,
    .max_out = 360 * deg,
};

// 底盘电机
inline M3508 w1({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .kt = KT,
    .is_invert = IS_INVERT,
});
inline M3508 w2({
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .kt = KT,
    .is_invert = IS_INVERT,
});
inline M3508 w3({
    .can_port = 1,
    .master_id = 0x203,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .kt = KT,
    .is_invert = IS_INVERT,
});
inline M3508 w4({
    .can_port = 1,
    .master_id = 0x204,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .kt = KT,
    .is_invert = IS_INVERT,
});

// 底盘
inline ChassisMecanum chassis({
                                  .chassis_radius = CHASSIS_RADIUS,
                                  .wheel_radius = WHEEL_RADIUS,
                                  .vxyz_pid_config = &vxyz_pid,
                                  .follow_pid_config = &follow_pid,
                              },
                              {w1, w2, w3, w4});