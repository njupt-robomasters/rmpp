#include "app.hpp"
#include "bsp/timer.hpp"
#include <algorithm>

// 操控参数
static constexpr float BULLET_SPEED = 20.0f;         // 弹速，m/s
static constexpr float BULLET_FREQ_INIT = 20.0f;     // 初始基础弹频，发/秒
static constexpr float BULLET_FREQ_STEP = 5.0f;      // C、Z 键每次增减的基础弹频，发/秒
static constexpr float BULLET_FREQ_MIN = 5.0f;       // 基础弹频下限，发/秒
static constexpr float BULLET_FREQ_MAX = 25.0f;      // 基础弹频上限，发/秒
static constexpr float DOUBLE_CLICK_TIME = 250 * ms; // 双击左键判定时间

// 跨周期状态
static float bullet_freq = BULLET_FREQ_INIT;
static Timer timer_click;
static bool has_click = false, is_force_shoot = false;
static bool last_left = false, last_z = false, last_c = false;

void shooter_loop() {
    shooter.set_enable(sentry_enable());

    // 遥控器：VT13 S 挡或 FSi6X SWC 下开摩擦轮，VT13 trigger 或 FSi6X SWB 下射击
    const bool radio_rub = vt13.mode == VT13::S || fsi6x.swc == FSi6X::DOWN;
    const bool radio_shoot = vt13.trigger || fsi6x.swb == FSi6X::DOWN;
    const bool radio_auto_aim = vt13.photo || fsi6x.swd == FSi6X::DOWN;

    // 遥控器射击时 pitch 摇杆作为弹频倍率，各遥控器只按自身射击开关决定 pitch 用途
    float radio_rate = 0;
    if (vt13.trigger)
        radio_rate += vt13.pitch;
    if (fsi6x.swb == FSi6X::DOWN)
        radio_rate += fsi6x.pitch;

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

    // 遥控器优先并按 pitch 倍率射击，键鼠次之，最后为自主模式视觉开火，后两者按基础弹频
    // 人工自瞄时需视觉开火许可，右键双击强制开火除外
    bool is_shoot = false;
    float rate = 0;
    if (radio_shoot && (!radio_auto_aim || mavlink.vision.is_fire)) {
        is_shoot = true;
        rate = radio_rate;
    } else if (mouse.left && (!mouse.right || is_force_shoot || mavlink.vision.is_fire)) {
        is_shoot = true;
        rate = 1;
    } else if (sentry_active() && mavlink.vision.is_fire) {
        is_shoot = true;
        rate = 1;
    }

    // 裁判系统识别为步兵时，C、Z 键按下一次增减基础弹频
    const int step = static_cast<int>(vt13.key.c && !last_c) - static_cast<int>(vt13.key.z && !last_z);
    last_c = vt13.key.c;
    last_z = vt13.key.z;
    if (referee.robot.type == Referee::INFANTRY)
        bullet_freq = std::clamp(bullet_freq + step * BULLET_FREQ_STEP, BULLET_FREQ_MIN, BULLET_FREQ_MAX);

    // 比赛中自主模式常开摩擦轮
    shooter.set_rub(radio_rub || (sentry_active() && sentry_in_game()));
    shooter.set_shoot(is_shoot);
    shooter.set_bullet_speed(BULLET_SPEED);
    shooter.set_bullet_freq(bullet_freq * rate);
    shooter.set_heat_remain(referee.shooter.heat_remain);
    shooter.update();

    // UI 显示基础弹频
    ui.bullet = bullet_freq;
    ui.bullet_is_speed = false;
}
