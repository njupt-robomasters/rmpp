#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "math/math.hpp"
#include "minipc.hpp"
#include <algorithm>
#include <cmath>

// VT13：摇杆叠加键盘斜坡，速度为 IMU 头的云台系，断联时为 0
static void control_by_vt13() {
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
}

// 自主：过洞随头，其余小陀螺；方向反馈掉线停车
static void control_by_auto() {
    if (!yaw1.is_connect || !yaw2.is_connect) {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(0, 0, 0);
        return;
    }

    const float vx = nav.vx + vt13.vx;
    const float vy = nav.vy + vt13.vy;
    if (nav.is_through_hole) {
        chassis.set_mode(ChassisMatrix::FOLLOW_MODE);
        chassis.set_speed(vx, vy, 0);
    } else {
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        // 平移越快转速越低，合速度线性插值到小陀螺转速
        const float speed = std::clamp(std::hypot(vx, vy), 0.0f, vt13.config.vxy_max);
        chassis.set_speed(vx, vy, map(speed, 0.0f, vt13.config.vxy_max, vt13.config.wr_max, WR_MIN));
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

    // 超级电容：使能或比赛中开 DCDC 并叠加放电补偿
    const bool is_cap_enable = vt13.is_enable || sentry_in_game();
    super_cap.set_enable(is_cap_enable);
    super_cap.set_boost_enable(is_cap_enable);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
