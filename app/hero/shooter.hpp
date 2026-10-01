#pragma once

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
    .motor = {.bus = {.can = Can(2), .tx_id = 0x21, .rx_id = 0x21}},
});
inline Vesc rub2({
    .motor = {.bus = {.can = Can(2), .tx_id = 0x22, .rx_id = 0x22}},
});
inline Vesc rub3({
    .motor = {.bus = {.can = Can(2), .tx_id = 0x23, .rx_id = 0x23}},
});
inline Vesc rub4({
    .motor = {.bus = {.can = Can(2), .tx_id = 0x24, .rx_id = 0x24}},
});

// 拨弹电机
inline Pid::Config shoot_angle_pid = {
    .kp = (3) / (6 * deg),
    .kd = (3) / (600 * deg_s),
    .max_out = 6,
};
inline DM4310 shoot({
    .motor = {
        .bus = {.can = Can(1), .tx_id = 0x11, .rx_id = 0x10},
        .install = {.offset = -11 * deg, .is_invert = true},
        .control = {.mode = Motor::SINGLE_TURN_POSITION_MODE, .pid_output_type = Motor::TORQUE_OUTPUT, .position_pid_config = &shoot_angle_pid},
    },
});

// 发射机构
inline Shooter42mm shooter({
                               .bullet_per_angle = 6 * (1.0f / rev),
                               .block_time = 500 * ms,
                               .reverse_time = 250 * ms,
                               .heat_protect = 100,
                           },
                           {rub1, rub2, rub3, rub4, shoot});

void shooter_loop();
