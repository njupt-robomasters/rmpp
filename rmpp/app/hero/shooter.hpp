#pragma once

#include "motor/VESC.hpp"
#include "motor/DM4310.hpp"
#include "module/shooter/Shooter_42mm.hpp"

// 用于Ozone调参
// #define shoot_angle_pid angle_pid
// #define shoot motor

// 摩擦轮
inline VESC rub1({
                     .can_port = 2,
                     .master_id = 0x21,
                     .slave_id = 0x21,
                 }, {});
inline VESC rub2({
                     .can_port = 2,
                     .master_id = 0x22,
                     .slave_id = 0x22,
                 }, {});
inline VESC rub3({
                     .can_port = 2,
                     .master_id = 0x23,
                     .slave_id = 0x23,
                 }, {});
inline VESC rub4({
                     .can_port = 2,
                     .master_id = 0x24,
                     .slave_id = 0x24,
                 }, {});

// 拨弹电机
inline PID::config_t shoot_angle_pid = {
    .kp = (10 * Nm) / (20 * deg),
    .kd = (10 * Nm) / (1000 * deg_s),
    .max_i = 3 * Nm,
    .max_out = 3 * Nm,
};
inline DM4310 shoot({
    .can_port = 1,
    .master_id = 0x10,
    .slave_id = 0x11,
    .reduction = DM4310::REDUCTION,
    .Kt = DM4310::Kt,
    .R = DM4310::R,
    .is_invert = true,
    .offset = 8 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .angle_pid_config = &shoot_angle_pid,
});

// 发射机构
inline Shooter_42mm shooter({
                                .bullet_per_angle = 6 * (default_unit / rev),
                            },
                            {rub1, rub2, rub3, rub4, shoot});