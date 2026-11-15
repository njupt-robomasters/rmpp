#pragma once

#include <cstdint>

#include "pid/pid.hpp"

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/el05.hpp"
#include "motor/vesc.hpp"
#include "robot/shooter/shooter_17mm.hpp"

// 摩擦轮：2×P630，CAN3，复用 Vesc 扩展帧协议，电机内部速度环
// tx_id/rx_id 为电调节点 ID；set_speed() 接收输出轴 rad/s，Vesc 按 reduction（含极对数）换算 ERPM
// 沿用节点 ID 0x21/0x22 与 Vesc 默认驱动参数，待实机核对极对数和安装方向；各 app 中发送暂时注释
inline Vesc rub1({
    .bus = {.can = Can(3), .tx_id = 0x21, .rx_id = 0x21},
    .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});
inline Vesc rub2({
    .bus = {.can = Can(3), .tx_id = 0x22, .rx_id = 0x22},
    .drive = {.max_current = Vesc::MAX_CURRENT, .kt = Vesc::KT, .reduction = Vesc::REDUCTION},
});

// 拨弹电机：灵足 EL05，CAN2 MIT 标准帧，控制 ID 0x12 / 反馈 ID 0x22；速度环 rad/s → Nm
inline Pid::Config shoot_pid = {
    .kp = (2.0f) / (60 * rpm),
    .max_out = 5.0f,
};
// 力矩限幅与 PID 限幅同为 5 Nm，max_current 为对应电流；发射机构以电流达到 max_current 判定卡弹
// 运行时可写发送周期，单位 ms，建议取 1~99，避免超过反馈超时
inline volatile uint32_t shoot_send_period_ms = 2;
inline EL05 shoot({
    .bus = {.can = Can(2), .tx_id = 0x12, .rx_id = 0x22},
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
