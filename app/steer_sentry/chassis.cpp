#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "math/math.hpp"
#include "minipc.hpp"
#include <algorithm>
#include <cmath>

// 导航目标点状态，退出自主时复位为去中心
static bool is_go_home = false;

// 导航目标点：默认去中心，收到有效血量后按阈值往返
static void set_nav_target() {
    if (referee.robot_status.maximum_HP != 0) {
        const uint16_t hp = referee.robot_status.current_HP;
        if (hp <= GO_HOME_HP)
            is_go_home = true;
        else if (hp >= GO_CENTER_HP)
            is_go_home = false;
    }

    nav.set_target(is_go_home ? HOME_X : CENTER_X, is_go_home ? HOME_Y : CENTER_Y);
}

// VT13：摇杆叠加键盘斜坡，速度为云台系，断联时为 0
static void control_by_vt13() {
    is_go_home = false;
    chassis.set_mode(ChassisSteer::DETACH_MODE);
    chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
}

// 自主：导航速度为大 yaw 雷达系，经负小 yaw 角转换到云台系，与 VT13 平移逐轴相加后限幅
// 过洞 1 随头对准枪口（未配跟随 PID，不旋转）、平移照常；过洞 0 小陀螺，平移越快转速越低，插值输入限幅
// 导航断联时导航量清零，摇杆仍可操作；任一 yaw 反馈丢失时停车
static void control_by_auto() {
    if (!yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisSteer::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }

    const float vxy_max = vt13.config.vxy_max, wr_max = vt13.config.wr_max;
    const auto [nav_vx, nav_vy] = rotate(nav.vx, nav.vy, -gimbal.yaw2.measure);
    const float vx = std::clamp(nav_vx + vt13.vx, -vxy_max, vxy_max);
    const float vy = std::clamp(nav_vy + vt13.vy, -vxy_max, vxy_max);
    if (nav.is_through_hole) {
        chassis.set_mode(ChassisSteer::FOLLOW_MODE);
        chassis.set_speed(vx, vy, vt13.wr);
    } else {
        const float speed = std::clamp(std::hypot(vx, vy), 0.0f, vxy_max);
        chassis.set_mode(ChassisSteer::DETACH_MODE);
        chassis.set_speed(vx, vy, std::clamp(map(speed, 0.0f, vxy_max, wr_max, WR_MIN) + vt13.wr, -wr_max, wr_max));
    }
}

void chassis_loop() {
    chassis.set_enable(vt13.is_enable);
    if (is_sentry_active()) {
        set_nav_target();
        control_by_auto();
    } else {
        control_by_vt13();
    }

    // 前进正方向为云台方向
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);

    // 未装超电：保持 OFF，按裁判功率上限与缓冲能量折算底盘功率目标
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
