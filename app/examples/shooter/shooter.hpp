#pragma once

#include "bsp/bsp.hpp"

#include "math/unit.hpp"
#include "module/shooter/shooter_17mm.hpp"
#include "motor/m2006.hpp"
#include "motor/vesc.hpp"

// 用于Ozone调参
// #define shoot_pid motor_speed_pid
// #define m_shoot motor

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

// 拨弹电机
inline Pid::Config shoot_pid = {
    .kp = (10) / (20 * rpm),
    .max_out = 10,
};
inline M2006 shoot({
    .can = bsp.can(1),
    .master_id = 0x206,
    .slave_id = 0x200,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &shoot_pid,
});

// 发射机构
inline Shooter17mm shooter({
                               .bullet_per_angle = 9.0f / 2.5f * (1.0f / rev),
                           },
                           {rub1, rub2, shoot});