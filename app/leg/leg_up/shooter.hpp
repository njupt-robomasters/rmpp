#pragma once

#include "motor/m3508.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter_17mm.hpp"

static constexpr float RUB_RELEASE_SPEED = 30 * rpm; // 摩擦轮关闭后低于此转速释放

// 摩擦轮
inline Vesc rub1({
    .can = Can(2),
    .tx_id = 0x30,
    .rx_id = 0x30,
    .max_current = M3508::MAX_CURRENT,
    .reduction = 7,
    .is_invert = false,
});
inline Vesc rub2({
    .can = Can(2),
    .tx_id = 0x31,
    .rx_id = 0x31,
    .max_current = M3508::MAX_CURRENT,
    .reduction = 7,
    .is_invert = false,
});

// 拨弹电机
inline Pid::Config shoot_pid_config = {
    .kp = 4.0f,
    .max_out = M3508::MAX_CURRENT,
};
inline M3508 shoot({
    .can = Can(2),
    .rx_id = 0x201,
    .is_invert = false,
});

// 发射机构：bullet_per_angle 按拨盘结构标定
inline Shooter17mm shooter({rub1, rub2, shoot},
                           {
                               .shoot_pid_config = &shoot_pid_config,
                               .bullet_per_angle = 10.0f / 56.0f * 22.0f / rev,
                               .heat_protect = 80, // 剩余热量低于此值停止拨弹，待整定
                           });

void shooter_loop();
