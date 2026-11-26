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

// 自主：视觉开火许可时按设定弹频
static void control_by_auto(const bool is_fire) {
    shooter.set_bullet_speed(vt13.config.bullet_speed);
    shooter.set_bullet_freq(is_fire ? vt13.config.bullet_freq : 0);
}

void shooter_loop() {
    const bool is_fire = aim.is_fire;

    // N 挡只允许底盘和云台使能，发射机构失能
    shooter.set_enable(sentry_enable() && vt13.rc.mode != VT13::N);
    if (sentry_in_game() || fn_latched())
        control_by_auto(is_fire);
    else
        control_by_vt13(is_fire);

    shooter.set_heat_remain(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();
}
