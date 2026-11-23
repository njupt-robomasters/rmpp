#include "app.hpp"
#include "bsp/timer.hpp"
#include <algorithm>

// 操控参数
static constexpr float VXY_MAX = 5.0f;     // 平移最大速度，m/s；100W下2.3m/s，不限功率3.2m/s
static constexpr float WR_MAX = 100 * rpm; // 旋转最大角速度，rad/s；100W下107rpm，不限功率155rpm
static constexpr float NAV_WR = 60 * rpm;  // 导航自动小陀螺角速度，rad/s
static constexpr float AXY = 100.0f;       // 键盘平移加速度，m/s^2
static constexpr float DXY = 100.0f;       // 键盘平移减速度，m/s^2
static constexpr float AR = 1000 * rpm;    // 键盘旋转角加速度，rad/s^2
static constexpr float DR = 1000 * rpm;    // 键盘旋转角减速度，rad/s^2

// 跨周期状态
static Timer loop_timer;
static float key_vx = 0, key_vy = 0, key_wr = 0;

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

    const bool is_enable = sentry_enable();
    chassis.set_enable(is_enable);

    // 键盘：WASD 平移，Shift、Ctrl 旋转；断联时直接清零
    if (vt13.is_connect) {
        key_vx = key_speed(key_vx, vt13.key.w, vt13.key.s, VXY_MAX, AXY, DXY, dt);
        key_vy = key_speed(key_vy, vt13.key.a, vt13.key.d, VXY_MAX, AXY, DXY, dt);
        key_wr = key_speed(key_wr, vt13.key.shift, vt13.key.ctrl, WR_MAX, AR, DR, dt);
    } else {
        key_vx = key_vy = key_wr = 0;
    }

    // 人工输入优先接管速度；人工与导航平移均以最终 IMU 头为前方
    float vx = (vt13.x + fsi6x.x) * VXY_MAX + key_vx;
    float vy = (vt13.y + fsi6x.y) * VXY_MAX + key_vy;
    float wr = vt13.wheel * WR_MAX + key_wr;
    const bool has_manual_input = vt13.x != 0 || vt13.y != 0 || vt13.wheel != 0 ||
                                  fsi6x.x != 0 || fsi6x.y != 0 ||
                                  key_vx != 0 || key_vy != 0 || key_wr != 0;
    auto control_mode = ChassisMatrix::DETACH_MODE;
    if (is_enable && sentry_active()) {
        // 有效 flag 选择旋转模式，人工平移保持该模式；离线时停止自动控制
        if (nav.is_connect && yaw1.is_connect && yaw2.is_connect) {
            if (!has_manual_input) {
                vx = nav.command.vx;
                vy = nav.command.vy;
            }
            if (nav.command.action_flag == 1) {
                control_mode = ChassisMatrix::FOLLOW_MODE;
            } else if (wr == 0) {
                // 人工主动旋转优先，否则保持自动小陀螺转速
                wr = NAV_WR;
            }
        }
    }
    chassis.set_mode(control_mode);
    chassis.set_speed(std::clamp(vx, -VXY_MAX, VXY_MAX), std::clamp(vy, -VXY_MAX, VXY_MAX), std::clamp(wr, -WR_MAX, WR_MAX));

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 超电：裁判离线或遥控显式关闭时 OFF；VT13 N/S 挡和比赛使用 BOOST
    // 自检和倒计时默认 CHARGE；FSi6X SWC 上/中/下对应 OFF/CHARGE/BOOST
    const bool is_cap_off = (fsi6x.is_connect && fsi6x.swc == FSi6X::UP) ||
                            (vt13.is_connect && vt13.mode == VT13::C);
    const auto progress = referee.game_status.game_progress;
    SuperCap::Mode cap_mode = SuperCap::OFF;
    if (!referee.is_connect || is_cap_off) {
        cap_mode = SuperCap::OFF;
    } else if ((vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) || sentry_in_game()) {
        cap_mode = SuperCap::BOOST;
    } else if (progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC) {
        cap_mode = SuperCap::CHARGE;
    } else if (fsi6x.is_connect && fsi6x.swc == FSi6X::DOWN) {
        cap_mode = SuperCap::BOOST;
    } else if (fsi6x.is_connect && fsi6x.swc == FSi6X::MID) {
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
