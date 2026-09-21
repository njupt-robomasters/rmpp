#pragma once

#include "math/Conversions.hpp"
#include "module/shooter/Shooter_17mm.hpp"
#include "motor/M3508.hpp"
#include "motor/VESC.hpp"

using math::rev;
using math::rpm;

// 用于Ozone调参
// #define shoot_pid motor_speed_pid
// #define shoot motor

// 摩擦轮
inline VESC rub1({
                     .can_port = 2,
                     .master_id = 0x21,
                     .slave_id = 0x21,
                 },
                 {});

inline VESC rub2({
                     .can_port = 2,
                     .master_id = 0x22,
                     .slave_id = 0x22,
                 },
                 {});

// 拨弹电机
inline PID::config_t shoot_pid = {
    .kp = (20) / (40 * rpm),
    .max_out = 20,
};
inline M3508 shoot({
    .can_port = 1,
    .master_id = 0x206,
    .slave_id = 0x200,
    .reduction = 268.0f / 17.0f,
    .Kt = M3508::Kt / M3508::REDUCTION * (268.0f / 17.0f),
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &shoot_pid,
});

// 发射机构
inline Shooter_17mm shooter({
                                .bullet_per_angle = 9.0f * (41.0f / 50.0f) * (1.0f / rev),
                            },
                            {rub1, rub2, shoot});