#pragma once

#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/unit.hpp"
#include "misc.hpp"
#include "shooter.hpp"

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
inline bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

// fn 开关：FSi6X 优先取 SWA 下；VT13 在 N、S 挡时 fn 键置位、pause 键清除
// 底盘、云台、发射共用同一锁存状态，同一周期内重复调用结果不变
inline bool rc_fn() {
    static bool is_fn = false;
    if (fsi6x.is_connect) {
        is_fn = fsi6x.swa == FSi6X::DOWN;
    } else if (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) {
        if (vt13.fn)
            is_fn = true;
        if (vt13.pause)
            is_fn = false;
    } else {
        is_fn = false;
    }
    return is_fn;
}

inline bool sentry_in_game() {
    return referee.game_status.game_progress == 4 && (referee.robot_status.robot_id == 7 || referee.robot_status.robot_id == 107);
}

inline bool sentry_active() {
    return sentry_in_game() || rc_fn();
}

// 自检、倒计时和比赛中遥控器断联也保持使能
inline bool sentry_enable() {
    const auto progress = referee.game_status.game_progress;
    return rc_enable() || progress == 2 || progress == 3 || progress == 4;
}
