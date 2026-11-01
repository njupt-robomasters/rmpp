#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/m2006.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 用于Ozone调参
// #define shoot_pid motor_speed_pid
// #define m_shoot motor

// 摩擦轮
inline Vesc rub1({
    .bus = {.can = Can(2), .tx_id = 0x21, .rx_id = 0x21}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub2({
    .bus = {.can = Can(2), .tx_id = 0x22, .rx_id = 0x22}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});

// 拨弹电机
inline Pid::Config shoot_pid = {
    .kp = (10) / (20 * rpm),
    .max_out = 10,
};
inline M2006 shoot({
    .bus = {.can = Can(1), .rx_id = 0x206},
    .drive = {.max_current = M2006::MAX_CURRENT, .kt = M2006::KT, .reduction = M2006::REDUCTION},
    .install = {.is_invert = false},
});

// 发射机构
inline Shooter17mm shooter({
                               .rub_pid_config = &shoot_pid,
                               .shoot_pid_config = &shoot_pid,
                               .bullet_per_angle = 9.0f / 2.5f * (1.0f / rev),
                               .heat_protect = 80,
                           },
                           {rub1, rub2, shoot});

void shooter_loop();

