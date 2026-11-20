#include "app.hpp"
#include "bsp/timer.hpp"
#include <algorithm>

// 操控参数
static constexpr float BULLET_SPEED = 5.0f;          // 弹速，m/s
static constexpr float BULLET_FREQ_INIT = 20.0f;     // 初始基础弹频，发/秒
static constexpr float BULLET_FREQ_STEP = 5.0f;      // C、Z 键每次增减的基础弹频，发/秒
static constexpr float BULLET_FREQ_MIN = 5.0f;       // 基础弹频下限，发/秒
static constexpr float BULLET_FREQ_MAX = 25.0f;      // 基础弹频上限，发/秒
static constexpr float DOUBLE_CLICK_TIME = 250 * ms; // 双击左键判定时间

// 跨周期状态
static float bullet_freq = BULLET_FREQ_INIT;
static Timer click_timer;
static bool has_click = false, is_force_shoot = false;
static bool last_left = false, last_z = false, last_c = false;

void shooter_loop() {
    const bool is_shooter_enable = sentry_enable() && !(vt13.is_connect && vt13.mode == VT13::N);
    shooter.set_enable(is_shooter_enable);

    // VT13 在线时按三挡手动控制；S 挡按住扳机后，左摇杆上下控制拨盘
    const bool is_vt13_feed = vt13.is_connect && vt13.mode == VT13::S && vt13.trigger;
    const bool radio_rub = fsi6x.swc == FSi6X::DOWN;
    const bool radio_shoot = fsi6x.swb == FSi6X::DOWN;
    const bool radio_auto_aim = fsi6x.swd == FSi6X::DOWN;

    // 遥控器射击时 pitch 摇杆作为弹频倍率，各遥控器只按自身射击开关决定 pitch 用途
    float radio_rate = 0;
    if (fsi6x.swb == FSi6X::DOWN)
        radio_rate += fsi6x.pitch;

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

    // VT13 在线时由扳机和左摇杆决定拨盘目标；其余入口沿用 FSi6X、键鼠与自主模式
    // 无运行请求时倍率为 0，负倍率反转拨盘
    float rate = 0;
    if (vt13.is_connect) {
        if (is_vt13_feed)
            rate = std::clamp(vt13.pitch, -1.0f, 1.0f);
    } else if (radio_shoot && (!radio_auto_aim || aim.vision.is_fire))
        rate = radio_rate;
    else if (mouse.left && (!mouse.right || is_force_shoot || aim.vision.is_fire))
        rate = 1;
    else if (sentry_active() && aim.vision.is_fire)
        rate = 1;

    // 裁判系统识别为步兵（红 3、蓝 103）时，C、Z 键按下一次增减基础弹频；哨兵身份下不调整
    const bool is_infantry = referee.robot_status.robot_id == 3 || referee.robot_status.robot_id == 103;
    if (is_infantry) {
        const int step = static_cast<int>(vt13.key.c && !last_c) - static_cast<int>(vt13.key.z && !last_z);
        bullet_freq = std::clamp(bullet_freq + step * BULLET_FREQ_STEP, BULLET_FREQ_MIN, BULLET_FREQ_MAX);
    }
    last_c = vt13.key.c;
    last_z = vt13.key.z;

    // 比赛中自主模式常开摩擦轮
    const bool is_rub = vt13.is_connect ? vt13.mode == VT13::S : radio_rub || (sentry_active() && sentry_in_game());
    shooter.set_bullet_speed(is_shooter_enable && is_rub ? BULLET_SPEED : 0.0f);
    shooter.set_bullet_freq(is_shooter_enable ? bullet_freq * rate : 0.0f);
    shooter.set_heat_remain(static_cast<int32_t>(referee.robot_status.shooter_barrel_heat_limit) -
                            referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();

    // UI 显示基础弹频
    ui.bullet = bullet_freq;
    ui.bullet_is_speed = false;
}
