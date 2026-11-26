#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include <algorithm>

// VT13：摇杆叠加键盘斜坡，速度为 IMU 头的云台系，断联时为 0
static void control_by_vt13() {
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
}

// 自主：导航速度为大 yaw 雷达系，经负小 yaw 角转到云台头部系后与 VT13 平移相加；导航掉线时只取遥控平移，方向反馈掉线时停车
// 导航测试阶段：过洞 1 随头对准枪口，过洞 0 分离模式、旋转输入为 0；平移照常
static void control_by_auto() {
    if (!yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }
    if (!nav.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(vt13.vx, vt13.vy, 0);
        return;
    }

    const float vxy_max = vt13.config.vxy_max;
    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    const float vx = std::clamp(nav_vx + vt13.vx, -vxy_max, vxy_max);
    const float vy = std::clamp(nav_vy + vt13.vy, -vxy_max, vxy_max);
    if (nav.is_through_hole)
        chassis.set_mode(ChassisMatrix::FOLLOW_MODE);
    else
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_speed(vx, vy, 0);
}

void chassis_loop() {
    chassis.set_enable(sentry_enable());
    if (sentry_active())
        control_by_auto();
    else
        control_by_vt13();

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 超电：裁判离线或 VT13 C 挡 OFF；VT13 N/S 挡和比赛 BOOST；自检和倒计时 CHARGE；其余 OFF
    const auto progress = referee.game_status.game_progress;
    if (!referee.is_connect || vt13.rc.mode == VT13::C)
        super_cap.set_mode(SuperCap::OFF);
    else if (vt13.is_enable || sentry_in_game())
        super_cap.set_mode(SuperCap::BOOST);
    else if (progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC)
        super_cap.set_mode(SuperCap::CHARGE);
    else
        super_cap.set_mode(SuperCap::OFF);

    // 功率上限与缓冲能量来自裁判系统；离线时沿用最后值，启动默认 120 W / 60 J
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
