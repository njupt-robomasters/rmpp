#pragma once

#include "bsp/kv_store.hpp"
#include "bsp/rgb.hpp"
#include "melody/melody.hpp"
#include "minipc/aim.hpp"
#include "minipc/nav.hpp"
#include "rc/vt13.hpp"
#include "referee.hpp"

// 板载 WS2812 与蜂鸣器
inline Rgb rgb;
inline Melody melody({});

// 持久化参数（W25Q64），当前只保存大 yaw 零位 yaw_offset
inline KvStore kv_store("jelly_sentry");

// 遥控器：VT13 图传接 USART1（921600）
// 平移 100W 下 2.3m/s、不限功率 3.2m/s，旋转 100W 下 107rpm、不限功率 155rpm；步兵身份下 C、Z 键调节最高弹频
inline VT13 vt13({
    .uart = Uart(1),
    .vxy_max = 5.0f,
    .axy = 100.0f,
    .dxy = 100.0f,
    .wr_max = 120 * rpm,
    .ar = 1000 * rpm,
    .dr = 1000 * rpm,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 5.0f,
    .bullet_freq = 20.0f,
});

// 上位机独立通道：CDC1 自瞄，CDC2 导航（17 字节 MODBUS CRC 帧）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(2)});

// fn 开关：VT13 在 N、S 挡时 fn 键置位、pause 键清除，失能或断联时清除
// 底盘、云台、发射共用同一锁存状态，同一周期内重复调用结果不变
inline bool rc_fn() {
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

// 比赛中且裁判系统识别为哨兵（红 7、蓝 107）
inline bool sentry_in_game() {
    const auto id = referee.robot_status.robot_id;
    return referee.game_status.game_progress == Referee::GAMING && (id == 7 || id == 107);
}

// 自主控制：比赛中，或按下 fn 测试；否则由 VT13 控制
inline bool sentry_active() {
    return sentry_in_game() || rc_fn();
}

// 整车使能：在线 VT13 的 C 挡优先失能，断联时挡位为 ERR；自检、倒计时和比赛中遥控器断联也保持使能
inline bool sentry_enable() {
    if (vt13.rc.mode == VT13::C)
        return false;

    const auto progress = referee.game_status.game_progress;
    return vt13.is_enable || progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC || progress == Referee::GAMING;
}
