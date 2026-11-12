#pragma once

#include "motor/m3508.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 摩擦轮
inline Vesc rub1({
    .bus = {.can = Can(2), .tx_id = 0x30, .rx_id = 0x30},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
});
inline Vesc rub2({
    .bus = {.can = Can(2), .tx_id = 0x31, .rx_id = 0x31},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
});

// 拨弹电机
inline M3508 shoot({
    .bus = {.can = Can(2), .rx_id = 0x201},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
    .install = {.is_invert = false},
});

// 拨弹电机 PID 参数
inline Pid::Config shoot_pid_config = {
    .kp = M3508::MAX_CURRENT / (60 * rpm),
    .max_out = M3508::MAX_CURRENT,
};

// 发射机构
inline Shooter17mm shooter({rub1, rub2, shoot},
                           {
                               .shoot_pid_config = &shoot_pid_config,
                               .bullet_per_angle = 10.0f / rev,
                           });

void shooter_loop();
