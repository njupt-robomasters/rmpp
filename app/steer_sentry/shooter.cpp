#include "shooter.hpp"
#include "app.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float BULLET_FREQ_STEP = 5.0f;       // C、Z 键每次增减的最高弹频，发/秒
static constexpr float BULLET_FREQ_MIN = 5.0f;        // 最高弹频下限，发/秒
static constexpr float BULLET_FREQ_MAX = 25.0f;       // 最高弹频上限，发/秒
static constexpr float DOUBLE_CLICK_TIME = 250 * ms;  // 双击左键判定时间
static constexpr float RUB_RELEASE_SPEED = 120 * rpm; // 摩擦轮关闭后低于此转速释放

// 右键双击强制开火
static bool has_click = false, is_force_shoot = false;
static bool last_left = false, last_z = false, last_c = false;
static Timer click_timer; // 按下左键时重新开始

// VT13：S 挡开摩擦轮；trigger 按 pitch 摇杆倍率、鼠标左键按最高弹频，负弹频反转拨盘
// 人工自瞄需视觉开火许可；按住右键时 DOUBLE_CLICK_TIME 内两次按下左键强制开火，松开左键或右键退出
// C、Z 键按下一次增减最高弹频，下一周期由 VT13 生效
static void control_by_vt13(const bool is_fire) {
    // 每周期让点击记录过期，避免长按右键跨过计数器回绕后误判双击
    const auto& mouse = vt13.mouse;
    if (has_click && click_timer.elapsed() >= DOUBLE_CLICK_TIME)
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

    const int step = static_cast<int>(vt13.key.c && !last_c) - static_cast<int>(vt13.key.z && !last_z);
    vt13.config.bullet_freq = std::clamp(vt13.config.bullet_freq + step * BULLET_FREQ_STEP, BULLET_FREQ_MIN, BULLET_FREQ_MAX);
    last_c = vt13.key.c;
    last_z = vt13.key.z;

    const bool can_fire = !vt13.is_aim || is_force_shoot || is_fire;
    shooter.set_bullet_speed(vt13.bullet_speed);
    shooter.set_bullet_freq(can_fire ? vt13.bullet_freq : 0);
}

// 自主：摩擦轮常开，视觉开火许可时按最高弹频
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

    shooter.set_heat_remain(static_cast<int32_t>(referee.robot_status.shooter_barrel_heat_limit) -
                            referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();

    // 摩擦轮关闭且低速时发零电流释放，避免 VESC 速度环在零速附近保持抖动
    if (shooter.bullet_speed.ref == 0) {
        for (Vesc* rub : {&rub1, &rub2})
            if (std::fabs(rub->speed) < RUB_RELEASE_SPEED)
                rub->set_current(0);
    }
}
