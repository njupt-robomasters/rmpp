#pragma once

#include "bsp/kv_store.hpp"
#include "melody/melody.hpp"
#include "rc/vt13.hpp"
#include "referee.hpp"

inline Melody melody({});

// 持久化参数，当前只保存大 yaw 零位 yaw_offset；沿用改名前的应用标记，保留已标定零位
inline KvStore kv_store("sentry");

// 遥控器：VT13 图传接 USART6
// 平移 100W 下 2.3m/s、不限功率 3.2m/s，旋转 100W 下 107rpm、不限功率 155rpm
inline VT13 vt13({
    .uart = Uart(6),
    .vxy_max = 5.0f,
    .axy = INFINITY,
    .dxy = INFINITY,
    .wr_max = 120 * rpm,
    .ar = INFINITY,
    .dr = INFINITY,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 20.0f,
    .bullet_freq = 10.0f,
});

// fn 开关：VT13 在 N、S 挡时 fn 键置位、pause 键清除，失能或断联时清除
// 场下调试哨兵自主行为用，底盘、云台、发射共用，同一周期内重复调用结果不变
inline bool is_sentry_test() {
    static bool is_fn = false;
    if (vt13.is_enable) {
        if (vt13.rc.fn)
            is_fn = true;
        if (vt13.rc.pause)
            is_fn = false;
    } else {
        is_fn = false;
    }
    return is_fn;
}

// 执行哨兵策略：场下测试模式，或裁判识别为哨兵（红 7、蓝 107）的正赛；比赛断联由 VT13 保持挡位
inline bool is_sentry_active() {
    const auto id = referee.robot_status.robot_id;
    return is_sentry_test() || (referee.game_status.game_progress == Referee::GAMING && (id == 7 || id == 107));
}
