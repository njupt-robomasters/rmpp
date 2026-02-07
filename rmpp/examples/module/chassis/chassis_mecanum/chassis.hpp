#pragma once

#include "motor/M3508.hpp"
#include "module/chassis/Chassis_Mecanum.hpp"

// 底盘电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr UnitFloat<Nm_A> Kt = M3508::Kt / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_INVERT = true;

// 底盘参数
static constexpr UnitFloat CHASSIS_RADIUS = 26.4f * cm;
static constexpr UnitFloat WHEEL_RADIUS = 7.6f * cm;
static constexpr UnitFloat<N> MAX_F = M3508::MAX_CURRENT * Kt / WHEEL_RADIUS * 4;

// 底盘PID参数
inline PID::config_t vxyz_pid = {
    .kp = MAX_F / (1 * (m / s)),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10 * Hz,
};

// 底盘电机
inline M3508 w1({
    .can_port = 2,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
});
inline M3508 w2({
    .can_port = 2,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
});
inline M3508 w3({
    .can_port = 2,
    .master_id = 0x203,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
});
inline M3508 w4({
    .can_port = 2,
    .master_id = 0x204,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
});

// 底盘
inline Chassis_Mecanum chassis({
                                   .chassis_radius = CHASSIS_RADIUS,
                                   .wheel_radius = WHEEL_RADIUS,
                                   .vxyz_pid_config = &vxyz_pid,
                               },
                               {w1, w2, w3, w4});