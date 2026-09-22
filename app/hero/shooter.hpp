#pragma once

#include "bsp/bsp.hpp"

#include "math/unit.hpp"
#include "module/shooter/shooter_42mm.hpp"
#include "motor/dm4310.hpp"
#include "motor/vesc.hpp"

// 用于Ozone调参
// #define shoot_angle_pid angle_pid
// #define shoot motor

// 摩擦轮
inline Vesc rub1({
                     .can = bsp.can(2),
                     .master_id = 0x21,
                     .slave_id = 0x21,
                 },
                 {});
inline Vesc rub2({
                     .can = bsp.can(2),
                     .master_id = 0x22,
                     .slave_id = 0x22,
                 },
                 {});
inline Vesc rub3({
                     .can = bsp.can(2),
                     .master_id = 0x23,
                     .slave_id = 0x23,
                 },
                 {});
inline Vesc rub4({
                     .can = bsp.can(2),
                     .master_id = 0x24,
                     .slave_id = 0x24,
                 },
                 {});

// 拨弹电机
inline Pid::Config shoot_angle_pid = {
    .kp = (3) / (6 * deg),
    .kd = (3) / (600 * deg_s),
    .max_out = 6,
};
inline DM4310 shoot({
    .can = bsp.can(1),
    .master_id = 0x10,
    .slave_id = 0x11,
    .reduction = DM4310::REDUCTION,
    .kt = DM4310::KT,
    .r = DM4310::R,
    .is_invert = true,
    .offset = -11 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .angle_pid_config = &shoot_angle_pid,
});

// 发射机构
inline Shooter42mm shooter({
                               .bullet_per_angle = 6 * (1.0f / rev),
                               .block_time = 500 * ms,
                               .reverse_time = 250 * ms,
                           },
                           {rub1, rub2, rub3, rub4, shoot});