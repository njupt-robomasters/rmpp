#include "app.hpp"
#include "bsp/timer.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float BULLET_FREQ_STEP = 5.0f;       // C、Z 键每次增减的最高弹频，发/秒
static constexpr float BULLET_FREQ_MIN = 5.0f;        // 最高弹频下限，发/秒
static constexpr float BULLET_FREQ_MAX = 25.0f;       // 最高弹频上限，发/秒
static constexpr float DOUBLE_CLICK_TIME = 250 * ms;  // 双击左键判定时间
static constexpr float RUB_RELEASE_SPEED = 120 * rpm; // 摩擦轮关闭后低于此转速释放

// 跨周期状态
static Timer click_timer;
static bool has_click = false, is_force_shoot = false;
static bool last_left = false, last_z = false, last_c = false;

void shooter_loop() {
    shooter.set_enable(sentry_enable());

    // 按住右键时 DOUBLE_CLICK_TIME 内两次按下左键进入强制开火，松开左键或右键退出
    // 每周期让点击记录过期，避免长按右键跨过计数器回绕后误判双击
    const auto& mouse = vt13.mouse;
    if (has_click && click_timer.elapsed() >= DOUBLE_CLICK_TIME) {
        click_timer.restart();
        has_click = false;
    }
    if (!mouse.right) {
        is_force_shoot = has_click = false;
    } else {
        if (mouse.left && !last_left) {
            is_force_shoot = has_click && click_timer.elapsed() < DOUBLE_CLICK_TIME;
            click_timer.restart();
            has_click = true;
        }
        if (!mouse.left)
            is_force_shoot = false;
    }
    last_left = mouse.left;

    // C、Z 键按下一次增减最高弹频，下一周期由 VT13 生效
    const int step = static_cast<int>(vt13.key.c && !last_c) - static_cast<int>(vt13.key.z && !last_z);
    last_c = vt13.key.c;
    last_z = vt13.key.z;
    vt13.config.bullet_freq = std::clamp(vt13.config.bullet_freq + step * BULLET_FREQ_STEP, BULLET_FREQ_MIN, BULLET_FREQ_MAX);

    // VT13 trigger 按 pitch 倍率、鼠标左键按最高弹频射击，其次为自主模式视觉开火
    // 人工自瞄时需视觉开火许可，右键双击强制开火除外；负弹频反转拨弹
    float bullet_freq = 0;
    if (vt13.bullet_freq != 0 && (!vt13.is_auto_aim || is_force_shoot || aim.vision.is_fire))
        bullet_freq = vt13.bullet_freq;
    else if (sentry_active() && aim.vision.is_fire)
        bullet_freq = vt13.config.bullet_freq;

    // VT13 S 挡开摩擦轮，比赛中自主模式常开
    const bool is_rub = vt13.bullet_speed > 0 || (sentry_active() && sentry_in_game());
    shooter.set_bullet_speed(is_rub ? vt13.config.bullet_speed : 0.0f);
    shooter.set_bullet_freq(bullet_freq);
    shooter.set_heat_remain(static_cast<int32_t>(referee.robot_status.shooter_barrel_heat_limit) -
                            referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();

    // 摩擦轮关闭且低速时发零电流释放，避免 VESC 速度环在零速附近保持抖动
    if (!is_rub) {
        for (Vesc* rub : {&rub1, &rub2})
            if (std::fabs(rub->speed) < RUB_RELEASE_SPEED)
                rub->set_current(0);
    }
}
