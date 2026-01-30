#pragma once

#include "motor/VESC.hpp"
#include "motor/DM4310.hpp"
#include "module/shooter/Shooter_42mm.hpp"

// 摩擦轮
inline VESC rub_left1({
                          .can_port = 2,
                          .master_id = 0x20,
                          .slave_id = 0x20,
                      }, {});
inline VESC rub_right1({
                           .can_port = 2,
                           .master_id = 0x21,
                           .slave_id = 0x21,
                       }, {});
inline VESC rub_left2({
                          .can_port = 2,
                          .master_id = 0x22,
                          .slave_id = 0x22,
                      }, {});
inline VESC rub_right2({
                           .can_port = 2,
                           .master_id = 0x23,
                           .slave_id = 0x23,
                       }, {});

// 拨弹电机速度PID参数
inline PID::config_t shoot_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (10 * Nm) / (200 * rpm),
    .ki = (10 * Nm) / (50 * deg),
    .max_i = 10 * Nm,
    .max_out = 10 * Nm,
};
// 拨弹电机
inline DM4310 shoot({
    .can_port = 1,
    .master_id = 0x10,
    .slave_id = 0x11,
    .reduction = DM4310::reduction,
    .Kt = DM4310::Kt,
    .R = DM4310::R,
    .is_invert = true,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &shoot_pid,
});

// 发射机构
inline Shooter_42mm shooter({
                                .bullet_per_rev = 6 * (Hz / rps),
                            },
                            {rub_left1, rub_right1, rub_left2, rub_right2, shoot});