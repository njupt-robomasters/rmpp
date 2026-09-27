#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dji/m3508.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 用于Ozone调参
// #define shoot_pid motor_speed_pid
// #define shoot motor

// 摩擦轮
inline Vesc rub1({
    .protocol = {.can = Can(2), .master_id = 0x21, .slave_id = 0x21},
});

inline Vesc rub2({
    .protocol = {.can = Can(2), .master_id = 0x22, .slave_id = 0x22},
});

// 拨弹电机
inline Pid::Config shoot_pid = {
    .kp = (20) / (40 * rpm),
    .max_out = 20,
};
inline M3508 shoot({
    .protocol = {.can = Can(1), .master_id = 0x206, .slave_id = 0x200},
    .physical = {.kt = M3508::DEFAULT_CONFIG.physical.kt / M3508::DEFAULT_CONFIG.install.reduction * (268.0f / 17.0f)},
    .install = {.is_invert = false, .reduction = 268.0f / 17.0f},
    .control = {.control_mode = Motor::SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &shoot_pid},
});

// 发射机构
inline Shooter17mm shooter({
                               .bullet_per_angle = 9.0f * (41.0f / 50.0f) * (1.0f / rev),
                               .bullet_speed = 20.0f,
                               .bullet_freq = 20,
                               .heat_protect = 80,
                           },
                           {rub1, rub2, shoot});

void shooter_loop();
