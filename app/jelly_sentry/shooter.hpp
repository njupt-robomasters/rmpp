#pragma once

#include "motor/el05.hpp"
#include "motor/m3508.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter_17mm.hpp"

// 双传送带驱动（工程沿用 rub 名称）：2×P630，CAN2，VESC 扩展帧，电调内部速度环
// tx_id/rx_id 为电调节点 ID；set_speed() 接收输出轴 rad/s，Vesc 按 reduction（含极对数）换算 ERPM
// 沿用节点 ID 0x21/0x22 与驱动参数，待实机核对极对数
// 三路方向均在各实例的 is_invert 中显式配置，true 为反向
inline Vesc rub1({
    .can = Can(2),
    .tx_id = 0x21,
    .rx_id = 0x21,
    .max_current = M3508::MAX_CURRENT,
    .reduction = 7.0f,
    .is_invert = true,
});
inline Vesc rub2({
    .can = Can(2),
    .tx_id = 0x22,
    .rx_id = 0x22,
    .max_current = M3508::MAX_CURRENT,
    .reduction = 7.0f,
    .is_invert = false,
});

// 拨弹电机：灵足 EL05，CAN2 私有扩展帧，电机 ID 0x12 / 主机 ID 0x22；速度环 rad/s → Nm
// 电机侧 protocol_1=0；使能 ID 0x03002212，运行反馈 ID 0x02801222（无故障）
inline Pid::Config shoot_pid_config = {
    .kp = 5.0f / (60 * rpm),
    .max_out = 6.0f,
};
inline EL05 shoot({
    .can = Can(2),
    .tx_id = 0x12,
    .rx_id = 0x22,
    .max_current = 6.0f / 0.94f,
    .max_torque = 6.0f,
    .is_invert = true,
});

// 发射机构：EL05 反馈为输出轴角度
inline Shooter17mm shooter({rub1, rub2, shoot}, {
                                                    .shoot_pid_config = &shoot_pid_config,
                                                    .bullet_per_angle = 9.0f / rev,
                                                    .heat_protect = 100,
                                                });

void shooter_loop();
