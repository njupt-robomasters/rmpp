#include "task_chassis.h"
#include "app.hpp"

static uint32_t dwt_cnt;
static float dt;

static void handle_rc() {
    // 解析遥控器控制
    status.rc.vx = dj6.x * settings.vxy_max;
    status.rc.vy = dj6.y * settings.vxy_max;
    status.rc.rpm = -dj6.yaw * settings.rpm_max;

    // 右边摇杆
    if (dj6.right_switch == DJ6::UP) {
        // 开启自动导航，忽略遥控器断连
        status.is_nav_on = true;
        status.ignore_rc_disconnect = true;
    } else if (dj6.right_switch == DJ6::MID) {
        // 开启自动导航，不忽略遥控器断连
        status.is_nav_on = true;
        status.ignore_rc_disconnect = false;
    } else if (dj6.right_switch == DJ6::DOWN) {
        // 关闭自动导航，不忽略遥控器断连
        status.is_nav_on = false;
        status.ignore_rc_disconnect = false;
    }
}

static bool checkHome() {
    if (fabsf(nav.position_x - settings.home_x) > settings.center_range)
        return false;
    if (fabsf(nav.position_y - settings.home_y) > settings.center_range)
        return false;
    return true;
}

static bool checkCenter() {
    if (fabsf(nav.position_x - settings.center_x) > settings.center_range)
        return false;
    if (fabsf(nav.position_y - settings.center_y) > settings.center_range)
        return false;
    return true;
}

static void followNAV() {
    // 响应导航，不自转
    status.nav.vx = nav.vel_x;
    status.nav.vy = nav.vel_y;
    status.nav.rpm = 0;
}

static void notNAV() {
    // 不响应导航，自转
    status.nav.vx = 0;
    status.nav.vy = 0;
    status.nav.rpm = settings.rpm_max;
}

static void handle_nav() {
    if (status.is_nav_on) {
        if (referee.robot_hp < settings.go_home_hp) {
            status.nav_status = Status::GO_HOME;
        }

        switch (status.nav_status) {
            case Status::GO_CENTER:
                status.nav_target_x = settings.center_x;
                status.nav_target_y = settings.center_y;
                followNAV();
                if (checkCenter()) {
                    status.nav_status = Status::IN_CENTER;
                }
                break;

            case Status::IN_CENTER:
                notNAV();
                break;

            case Status::GO_HOME:
                status.nav_target_x = settings.home_x;
                status.nav_target_y = settings.home_y;
                followNAV();
                if (checkHome()) {
                    status.nav_status = Status::IN_HOME;
                }
                break;

            case Status::IN_HOME:
                notNAV();
                if (referee.robot_hp == referee.robot_hp_limit) {
                    status.nav_status = Status::GO_CENTER;
                }
                break;

            default:
                break;
        }
    } else {
        status.nav.vx = 0;
        status.nav.vy = 0;
        status.nav.rpm = 0;
    }
}

[[noreturn]] void task_chassis_entry(void const *argument) {
    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        handle_rc();
        handle_nav();

        // 检查遥控器连接
        if (dj6.is_connected == false && not status.ignore_rc_disconnect) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;;
        }

        // 合并遥控和自动导航控制
        const float vx = clamp(status.rc.vx + status.nav.vx, settings.vxy_max);
        const float vy = clamp(status.rc.vy + status.nav.vy, settings.vxy_max);
        const float rpm = clamp(status.rc.rpm + status.nav.rpm, settings.rpm_max);

        chassis.SetEnable(true); // 使能
        chassis.SetPowerLimit(referee.chassis_power_limit); // 底盘功率限制
        chassis.SetSpeed(vx, vy, rpm); // 设置速度

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
