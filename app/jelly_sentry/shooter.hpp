#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/el05.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 摩擦轮：2×P630，CAN2，电机内部速度环；驱动待写
// Shooter17mm 只接受 Vesc 摩擦轮，暂用 Vesc 占位：不调用 send()，总线上无对应状态帧，始终离线
// P630 收发 ID 不能落在 0x010~0x013、0x141、0x241 上
inline Vesc rub1({
    .bus = {.can = Can(2), .tx_id = 0x21, .rx_id = 0x21},
    .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub2({
    .bus = {.can = Can(2), .tx_id = 0x22, .rx_id = 0x22},
    .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});

// 拨弹电机：灵足 EL05，CAN3 扩展帧，出厂 ID 电机 0x7F / 主机 0xFD；速度环 rad/s → Nm
inline Pid::Config shoot_pid = {
    .kp = (2.0f) / (60 * rpm),
    .max_out = 5.0f,
};
// 力矩限幅与 PID 限幅同为 5 Nm，max_current 为对应电流；发射机构以电流达到 max_current 判定卡弹
inline EL05 shoot({
    .bus = {.can = Can(3), .tx_id = EL05::DEFAULT_MOTOR_ID, .rx_id = EL05::DEFAULT_HOST_ID},
    .drive = {.max_current = 5.0f / EL05::KT, .max_torque = 5.0f, .kt = EL05::KT, .reduction = EL05::REDUCTION},
    .install = {.is_invert = false},
});

// 发射机构：EL05 反馈为输出轴角度；TODO: bullet_per_angle 按拨盘结构修改
inline Shooter17mm shooter({rub1, rub2, shoot}, {
                                                    .shoot_pid_config = &shoot_pid,
                                                    .shoot_output = Shooter17mm::TORQUE_OUTPUT,
                                                    .bullet_per_angle = 9.0f / rev,
                                                    .heat_protect = 80,
                                                });

void shooter_loop();
