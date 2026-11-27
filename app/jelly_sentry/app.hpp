#pragma once

#include "bsp/kv_store.hpp"
#include "melody/melody.hpp"
#include "rc/vt13.hpp"
#include "referee.hpp"

// 板载蜂鸣器
inline Melody melody({});

// 持久化参数（W25Q64），当前只保存大 yaw 零位 yaw_offset
inline KvStore kv_store("jelly_sentry");

// 遥控器：VT13 图传接 USART1（921600）
// 平移 100W 下 2.3m/s、不限功率 3.2m/s，旋转 100W 下 107rpm、不限功率 155rpm；鼠标滚轮调节弹频
inline VT13 vt13({
    .uart = Uart(1),
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

// fn 锁存：VT13 在 N、S 挡时 fn 键置位、pause 键清除，失能或断联时清除
// 底盘、云台、发射共用同一锁存状态，同一周期内重复调用结果不变
inline bool fn_latched() {
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

// 赛场阶段：裁判系统识别为哨兵（红 7、蓝 107），处于自检、倒计时或正赛
inline bool sentry_official() {
    const auto id = referee.robot_status.robot_id;
    const auto progress = referee.game_status.game_progress;
    return (id == 7 || id == 107) &&
           (progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC || progress == Referee::GAMING);
}

// 比赛中且裁判系统识别为哨兵（红 7、蓝 107）
inline bool sentry_in_game() {
    const auto id = referee.robot_status.robot_id;
    return referee.game_status.game_progress == Referee::GAMING && (id == 7 || id == 107);
}

// 整车使能：赛场自检、倒计时和正赛强制使能（屏蔽操作间默认 C 挡）；场外 C 挡优先失能
inline bool sentry_enable() {
    if (sentry_official())
        return true;
    if (vt13.rc.mode == VT13::C)
        return false;

    const auto progress = referee.game_status.game_progress;
    return vt13.is_enable || progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC || progress == Referee::GAMING;
}
