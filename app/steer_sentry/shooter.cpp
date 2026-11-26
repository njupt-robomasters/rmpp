#include "shooter.hpp"
#include "app.hpp"

// VT13：S 挡开摩擦轮；trigger 按 pitch 摇杆倍率、鼠标左键按设定弹频，负弹频反转拨盘
// 人工自瞄需视觉开火许可
static void control_by_vt13(const bool is_fire) {
    const bool can_fire = !vt13.is_aim || is_fire;
    shooter.set_bullet_speed(vt13.bullet_speed);
    shooter.set_bullet_freq(can_fire ? vt13.bullet_freq : 0);
}

// 自主：摩擦轮常开，视觉开火许可时按设定弹频
static void control_by_auto(const bool is_fire) {
    shooter.set_bullet_speed(vt13.config.bullet_speed);
    shooter.set_bullet_freq(is_fire ? vt13.config.bullet_freq : 0);
}

void shooter_loop(const bool is_fire) {
    // VT13 N 挡只允许底盘和云台使能，发射机构失能
    shooter.set_enable(sentry_enable() && vt13.rc.mode != VT13::N);
    if (sentry_active())
        control_by_auto(is_fire);
    else
        control_by_vt13(is_fire);

    shooter.set_heat_remain(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();
}
