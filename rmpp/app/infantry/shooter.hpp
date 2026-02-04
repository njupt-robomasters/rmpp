#pragma once

#include "motor/VESC.hpp"
#include "motor/M2006.hpp"
#include "module/shooter/Shooter_17mm.hpp"

// 摩擦轮
inline VESC rub_left({
                         .can_port = 2,
                         .master_id = 0x20,
                         .slave_id = 0x20,
                     }, {});

inline VESC rub_right({
                          .can_port = 2,
                          .master_id = 0x21,
                          .slave_id = 0x21,
                      }, {});

// 拨弹电机PID参数
inline PID::config_t shoot_pid = {
    .kp = (10 * A) / (20 * rpm),
    .max_out = 10 * A,
};
// 拨弹电机
inline M2006 shoot({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x200,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &shoot_pid,
});

// 发射机构
inline Shooter_17mm shooter({
                                .bullet_per_rev = 9.0f / 2.5f * (Hz / rev),
                            },
                            {rub_left, rub_right, shoot});