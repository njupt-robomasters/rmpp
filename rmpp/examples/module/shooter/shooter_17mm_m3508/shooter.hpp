#pragma once

#include "motor/VESC.hpp"
#include "motor/M3508.hpp"
#include "module/shooter/Shooter_17mm.hpp"

// 摩擦轮
inline VESC rub_left({
    .can_port = 2,
    .master_id = 0x20,
    .slave_id = 0x20,
},
{});

inline VESC rub_right({
    .can_port = 2,
    .master_id = 0x21,
    .slave_id = 0x21,
},
{});

// 拨弹电机速度PID参数
inline PID::config_t shoot_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (80 * rpm),
    .max_out = 20 * A,
    .fc = 1 * Hz,
};
// 拨弹电机
inline M3508 shoot({
    .can_port = 1,
    .master_id = 0x206,
    .slave_id = 0x200,
    .reduction = M3508::reduction,
    .Kt = M3508::Kt,
    .R = M3508::R,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &shoot_pid,
});

// 发射机构
inline Shooter_17mm shooter({
    .bullet_per_rev = 9.0f * (41.0f / 50.0f) * (Hz / rps),
},
{rub_left, rub_right, shoot});