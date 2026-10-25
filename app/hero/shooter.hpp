#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dm/dm4310.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_42mm.hpp"

// 用于Ozone调参
// #define shoot_angle_pid angle_pid
// #define shoot motor

// 摩擦轮
inline Vesc rub1({
    .bus = {.can = Can(2), .tx_id = 0x21, .rx_id = 0x21}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub2({
    .bus = {.can = Can(2), .tx_id = 0x22, .rx_id = 0x22}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub3({
    .bus = {.can = Can(2), .tx_id = 0x23, .rx_id = 0x23}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub4({
    .bus = {.can = Can(2), .tx_id = 0x24, .rx_id = 0x24}, .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});

// 拨弹电机
inline Pid::Config shoot_angle_pid = {
    .kp = (3) / (6 * deg),
    .kd = (3) / (600 * deg_s),
    .max_out = 6,
};
inline Pid::Config rub_speed_pid = {
    .kp = 1,
    .max_out = 20,
};
inline DM4310 shoot({
    .bus = {.can = Can(1), .tx_id = 0x11, .rx_id = 0x10},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.offset = -11 * deg, .is_invert = true},
});

// 发射机构
inline Shooter42mm shooter({
                               .rub_pid_config = &rub_speed_pid,
                               .shoot_pid_config = &shoot_angle_pid,
                               .shoot_output = Shooter::TORQUE_OUTPUT,
                               .bullet_per_angle = 6 * (1.0f / rev),
                               .block_time = 500 * ms,
                               .reverse_time = 250 * ms,
                               .heat_protect = 100,
                           },
                           {rub1, rub2, rub3, rub4, shoot});

void shooter_loop();
