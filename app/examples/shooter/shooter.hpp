#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dji/c610.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 用于Ozone调参
// #define shoot_pid motor_speed_pid
// #define m_shoot motor

// 摩擦轮
inline Vesc rub1({
    .motor = {.bus = {.can = Can(2), .tx_id = 0x21, .rx_id = 0x21}},
});
inline Vesc rub2({
    .motor = {.bus = {.can = Can(2), .tx_id = 0x22, .rx_id = 0x22}},
});

// 拨弹电机
inline Pid::Config shoot_pid = {
    .kp = (10) / (20 * rpm),
    .max_out = 10,
};
inline C610 shoot({
    .bus = {.can = Can(1), .rx_id = 0x206},
    .install = {.is_invert = false},
    .control = {.mode = Motor::SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &shoot_pid},
});

// 发射机构
inline Shooter17mm shooter({
                               .bullet_per_angle = 9.0f / 2.5f * (1.0f / rev),
                           },
                           {rub1, rub2, shoot});
