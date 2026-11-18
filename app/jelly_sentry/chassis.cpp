#include "app.hpp"
#include "bsp/timer.hpp"
#include "math/math.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float VXY_MAX = 5.0f;     // 平移最大速度，m/s；100W下2.3m/s，不限功率3.2m/s
static constexpr float WR_MAX = 120 * rpm; // 旋转最大角速度，rad/s；100W下107rpm，不限功率155rpm
static constexpr float WR_MIN = 60 * rpm;  // 导航满速平移时的小陀螺角速度，与底盘功率控制保留转速一致
static constexpr float AXY = 100.0f;       // 键盘平移加速度，m/s^2
static constexpr float DXY = 100.0f;       // 键盘平移减速度，m/s^2
static constexpr float AR = 1000 * rpm;    // 键盘旋转角加速度，rad/s^2
static constexpr float DR = 1000 * rpm;    // 键盘旋转角减速度，rad/s^2

// 自主导航参数：坐标 m，比赛中用比赛坐标，否则用调试坐标；FORCE_* 强制使用其中一套
static constexpr bool FORCE_GAME_POSITION = false;
static constexpr bool FORCE_TEST_POSITION = false;
static constexpr float GAME_CENTER_X = 4 - 2 + 0.2f, GAME_CENTER_Y = 6 - 1.5f + 0.2f;
static constexpr float GAME_HOME_X = 0, GAME_HOME_Y = 0;
static constexpr float TEST_CENTER_X = 1, TEST_CENTER_Y = 0;
static constexpr float TEST_HOME_X = 0, TEST_HOME_Y = 0;
static constexpr int GO_HOME_HP = 120, GO_CENTER_HP = 400; // 低于此血量回家、恢复到此血量去中心点

enum ChassisState { GO_CENTER,
                    GO_HOME };

// 跨周期状态
static ChassisState chassis_state = GO_CENTER;
static Timer loop_timer;
static float key_vx = 0, key_vy = 0, key_wr = 0;
static bool last_enable = false;
static ChassisMatrix::Mode mode = ChassisMatrix::DETACH_MODE;

// 按住正向或反向键时加速，松开时减速到 0
static float key_speed(const float speed, const bool positive, const bool negative, const float maximum, const float acceleration, const float deceleration, const float dt) {
    if (positive)
        return std::clamp(std::max(speed, 0.0f) + acceleration * dt, -maximum, maximum);
    if (negative)
        return std::clamp(std::min(speed, 0.0f) - acceleration * dt, -maximum, maximum);
    if (speed > 0)
        return std::max(0.0f, speed - deceleration * dt);
    return std::min(0.0f, speed + deceleration * dt);
}

void chassis_loop() {
    const float dt = loop_timer.restart();

    // 使能变化时回到分离模式，防止车突然转动伤人；fn 打开后切到跟随并保持到下次使能变化
    const bool is_enable = sentry_enable();
    if (is_enable != last_enable)
        mode = ChassisMatrix::DETACH_MODE;
    last_enable = is_enable;
    if (rc_fn())
        mode = ChassisMatrix::FOLLOW_MODE;
    chassis.set_enable(is_enable);
    chassis.set_mode(mode);

    // 键盘：WASD 平移，Shift、Ctrl 旋转；断联时直接清零
    if (vt13.is_connect) {
        key_vx = key_speed(key_vx, vt13.key.w, vt13.key.s, VXY_MAX, AXY, DXY, dt);
        key_vy = key_speed(key_vy, vt13.key.a, vt13.key.d, VXY_MAX, AXY, DXY, dt);
        key_wr = key_speed(key_wr, vt13.key.shift, vt13.key.ctrl, WR_MAX, AR, DR, dt);
    } else {
        key_vx = key_vy = key_wr = 0;
    }

    // 自主导航速度：雷达装在大 yaw 上，导航速度定义在大 yaw 坐标系，按小 yaw 相对大 yaw 的角度旋转到云台系
    // 小陀螺按平移速度在最大转速和保留转速间插值，导航越快转得越慢
    float vx = 0, vy = 0, wr = 0;
    if (sentry_active()) {
        const bool game_position = FORCE_GAME_POSITION || (!FORCE_TEST_POSITION && sentry_in_game());
        if (chassis_state == GO_CENTER) {
            nav.target_position.x = game_position ? GAME_CENTER_X : TEST_CENTER_X;
            nav.target_position.y = game_position ? GAME_CENTER_Y : TEST_CENTER_Y;
            if (referee.robot_status.current_HP <= GO_HOME_HP)
                chassis_state = GO_HOME;
        } else {
            nav.target_position.x = game_position ? GAME_HOME_X : TEST_HOME_X;
            nav.target_position.y = game_position ? GAME_HOME_Y : TEST_HOME_Y;
            if (referee.robot_status.current_HP >= GO_CENTER_HP)
                chassis_state = GO_CENTER;
        }
        std::tie(vx, vy) = rotate(nav.chassis_speed.vx, nav.chassis_speed.vy, gimbal.yaw2.measure);
        const float speed = std::hypotf(vx, vy);
        wr = map(speed, 0.0f, VXY_MAX, WR_MAX, WR_MIN);
    } else {
        chassis_state = GO_CENTER;
        nav.target_position = {};
    }

    // 叠加摇杆与键盘，断联时摇杆已复位为 0
    vx += (vt13.x + fsi6x.x) * VXY_MAX + key_vx;
    vy += (vt13.y + fsi6x.y) * VXY_MAX + key_vy;
    wr += vt13.wheel * WR_MAX + key_wr;
    chassis.set_speed(std::clamp(vx, -VXY_MAX, VXY_MAX), std::clamp(vy, -VXY_MAX, VXY_MAX), std::clamp(wr, -WR_MAX, WR_MAX));

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 超电：裁判离线或任一在线遥控器显式关闭时 OFF；比赛 BOOST，自检和倒计时 CHARGE
    // 人工测试：FSi6X SWC 上/中/下、VT13 C/N/S 对应 OFF/CHARGE/BOOST
    const bool is_cap_off = (fsi6x.is_connect && fsi6x.swc == FSi6X::UP) ||
                            (vt13.is_connect && vt13.mode == VT13::C);
    const auto progress = referee.game_status.game_progress;
    SuperCap::Mode cap_mode = SuperCap::OFF;
    if (!referee.is_connect || is_cap_off) {
        cap_mode = SuperCap::OFF;
    } else if (sentry_in_game()) {
        cap_mode = SuperCap::BOOST;
    } else if (progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC) {
        cap_mode = SuperCap::CHARGE;
    } else if ((fsi6x.is_connect && fsi6x.swc == FSi6X::DOWN) || (vt13.is_connect && vt13.mode == VT13::S)) {
        cap_mode = SuperCap::BOOST;
    } else if ((fsi6x.is_connect && fsi6x.swc == FSi6X::MID) || (vt13.is_connect && vt13.mode == VT13::N)) {
        cap_mode = SuperCap::CHARGE;
    }
    super_cap.set_mode(cap_mode);

    // 功率上限与缓冲能量来自裁判系统；离线时沿用最后值，启动默认 120 W / 60 J
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
