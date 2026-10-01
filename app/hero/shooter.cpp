#include "app.hpp"
#include "bsp/timer.hpp"

// 操控参数
static constexpr float BULLET_SPEED_INIT = 15.0f;    // 初始弹速，m/s
static constexpr float BULLET_SPEED_STEP = 0.1f;     // C、Z 键每次增减的弹速，m/s
static constexpr float DOUBLE_CLICK_TIME = 250 * ms; // 双击左键判定时间

// 跨周期状态
static float bullet_speed = BULLET_SPEED_INIT;
static Timer timer_click;
static bool has_click = false, is_force_shoot = false;
static bool last_left = false, last_z = false, last_c = false;

void shooter_loop() {
    shooter.set_enable(rc_enable());

    // 遥控器：VT13 S 挡或 FSi6X SWC 下开摩擦轮，VT13 trigger 或 FSi6X SWB 下射击
    const bool is_rub = vt13.mode == VT13::S || fsi6x.swc == FSi6X::DOWN;
    const bool radio_shoot = vt13.trigger || fsi6x.swb == FSi6X::DOWN;
    const bool radio_auto_aim = vt13.photo || fsi6x.swd == FSi6X::DOWN;

    // 按住右键时 DOUBLE_CLICK_TIME 内两次按下左键进入强制开火，松开左键或右键退出
    // 每周期让点击记录过期，避免长按右键跨过计数器回绕后误判双击
    const auto& mouse = vt13.mouse;
    if (has_click && timer_click.elapsed() >= DOUBLE_CLICK_TIME) {
        timer_click.restart();
        has_click = false;
    }
    if (!mouse.right) {
        is_force_shoot = has_click = false;
    } else {
        if (mouse.left && !last_left) {
            is_force_shoot = has_click && timer_click.elapsed() < DOUBLE_CLICK_TIME;
            timer_click.restart();
            has_click = true;
        }
        if (!mouse.left)
            is_force_shoot = false;
    }
    last_left = mouse.left;

    // 遥控器优先，键鼠次之；自瞄时需视觉开火许可，右键双击强制开火除外
    const bool is_shoot = (radio_shoot && (!radio_auto_aim || mavlink.vision.is_fire)) ||
                          (mouse.left && (!mouse.right || is_force_shoot || mavlink.vision.is_fire));

    // 裁判系统识别为英雄时，C、Z 键按下一次增减弹速
    const int step = static_cast<int>(vt13.key.c && !last_c) - static_cast<int>(vt13.key.z && !last_z);
    last_c = vt13.key.c;
    last_z = vt13.key.z;
    if (referee.robot.type == Referee::HERO)
        bullet_speed += step * BULLET_SPEED_STEP;

    shooter.set_rub(is_rub);
    shooter.set_shoot(is_shoot);
    shooter.set_bullet_speed(bullet_speed);
    shooter.set_heat_remain(referee.shooter.heat_remain);
    shooter.update();

    // UI 显示弹速
    ui.bullet = bullet_speed;
    ui.bullet_is_speed = true;
}
