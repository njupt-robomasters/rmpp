#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "math/math.hpp"
#include <algorithm>
#include <cmath>

static constexpr float WR_MIN = 60 * rpm; // 导航满速平移时的小陀螺角速度，rad/s

// VT13：摇杆叠加键盘斜坡，速度为云台系，断联时为 0
static void control_by_vt13() {
    chassis.set_mode(ChassisSteer::DETACH_MODE);
    chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
}

// 自主：导航速度为大 yaw 雷达系，经负小 yaw 角转换到云台系；导航或大小 yaw 离线时停止
// 过洞 1 随头对准枪口（未配跟随 PID，不旋转）、平移照常；过洞 0 小陀螺，平移越快转速越低，插值输入限幅
static void control_by_auto() {
    if (!nav.is_connect || !yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisSteer::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }

    const float vxy_max = vt13.config.vxy_max, wr_max = vt13.config.wr_max;
    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    const float vx = std::clamp(nav_vx, -vxy_max, vxy_max);
    const float vy = std::clamp(nav_vy, -vxy_max, vxy_max);
    if (nav.is_through_hole) {
        chassis.set_mode(ChassisSteer::FOLLOW_MODE);
        chassis.set_speed(vx, vy, 0);
    } else {
        const float speed = std::clamp(std::hypot(vx, vy), 0.0f, vxy_max);
        chassis.set_mode(ChassisSteer::DETACH_MODE);
        chassis.set_speed(vx, vy, map(speed, 0.0f, vxy_max, wr_max, WR_MIN));
    }
}

void chassis_loop() {
    chassis.set_enable(sentry_enable());
    if (sentry_active())
        control_by_auto();
    else
        control_by_vt13();

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 未装超电：保持 OFF，按裁判功率上限与缓冲能量折算底盘功率目标
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
