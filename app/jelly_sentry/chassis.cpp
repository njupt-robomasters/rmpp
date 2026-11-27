#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "math/math.hpp"
#include "minipc.hpp"
#include <algorithm>

// VT13：摇杆叠加键盘斜坡，速度为 IMU 头的云台系，断联时为 0
static void control_by_vt13() {
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
}

// 自主：雷达系导航速度转到云台系后叠加遥控平移；过洞随头，其余 90→60 rpm 小陀螺
// 导航断联时平移归零，过洞保持停车，其余原地小陀螺；任一 yaw 反馈掉线停车
static void control_by_auto() {
    if (!yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }

    if (!nav.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, is_nav_through_hole ? 0 : WR_MAX);
        return;
    }

    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    const float vx = nav_vx + vt13.vx;
    const float vy = nav_vy + vt13.vy;
    if (is_nav_through_hole) {
        chassis.set_mode(ChassisMatrix::FOLLOW_MODE);
        chassis.set_speed(vx, vy, 0);
    } else {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        // 平移越快转速越低，合速度线性插值到小陀螺转速
        const float speed = std::clamp(std::hypot(vx, vy), 0.0f, vt13.config.vxy_max);
        chassis.set_speed(vx, vy, map(speed, 0.0f, vt13.config.vxy_max, WR_MAX, WR_MIN));
    }
}

void chassis_loop() {
    chassis.set_enable(sentry_enable());
    if (sentry_in_game() || fn_latched())
        control_by_auto();
    else
        control_by_vt13();

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 超电：裁判离线或 C 挡关闭；遥控使能或比赛中放电补偿，自检和倒计时只充电
    bool is_cap_enable = false, is_boost_enable = false;
    if (referee.is_connect && vt13.rc.mode != VT13::C) {
        const auto progress = referee.game_status.game_progress;
        is_boost_enable = vt13.is_enable || sentry_in_game();
        is_cap_enable = is_boost_enable || progress == Referee::REFEREE_SELF_CHECK || progress == Referee::COUNTDOWN_5SEC;
    }
    super_cap.set_enable(is_cap_enable);
    super_cap.set_boost_enable(is_boost_enable);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
