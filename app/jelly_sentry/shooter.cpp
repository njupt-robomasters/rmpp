#include "shooter.hpp"
#include "app.hpp"
#include "minipc.hpp"

// 右键按住时双击左键强制开火，松开左键或右键退出
static bool has_click = false, is_force_shoot = false;
static bool last_left = false;
static Timer click_timer;

// VT13：人工自瞄需视觉开火许可；右键按住时 250 ms 内双击左键强制开火
static void control_by_vt13(const bool is_fire) {
    const auto& mouse = vt13.mouse;
    if (has_click && click_timer.elapsed() >= 250 * ms)
        has_click = false;
    if (!mouse.right) {
        is_force_shoot = has_click = false;
    } else {
        if (mouse.left && !last_left) {
            is_force_shoot = has_click;
            click_timer.restart();
            has_click = true;
        }
        if (!mouse.left)
            is_force_shoot = false;
    }
    last_left = mouse.left;

    const bool can_fire = !vt13.is_aim || is_force_shoot || is_fire;
    shooter.set_bullet_speed(vt13.bullet_speed);
    shooter.set_bullet_freq(can_fire ? vt13.bullet_freq : 0);
}

// 自主：摩擦轮常开；允许交战且视觉给出开火许可时按设定弹频，回家期间停拨弹
static void control_by_auto(const bool is_fire) {
    shooter.set_bullet_speed(vt13.config.bullet_speed);
    shooter.set_bullet_freq(sentry_can_engage() && is_fire ? vt13.config.bullet_freq : 0);
}

void shooter_loop() {
    const bool is_fire = aim.is_fire;

    // 发射机构：先服从整车使能；场外 N 挡只允许底盘和云台使能
    shooter.set_enable(sentry_enable() && (sentry_official() || vt13.rc.mode != VT13::N));
    if (sentry_in_game() || fn_latched()) {
        control_by_auto(is_fire);
    } else if (sentry_official()) {
        // 赛场自检停摩擦轮，5s 倒计时提前开启摩擦轮，拨盘均停转
        const bool is_countdown = referee.game_status.game_progress == Referee::COUNTDOWN_5SEC;
        shooter.set_bullet_speed(is_countdown ? vt13.config.bullet_speed : 0);
        shooter.set_bullet_freq(0);
    } else {
        control_by_vt13(is_fire);
    }

    shooter.set_heat(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat,
                     referee.robot_status.shooter_barrel_cooling_value);
    shooter.update();
}
