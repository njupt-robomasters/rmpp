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
        // 关闭自动导航，不忽略遥控器断连
        status.is_nav_on = false;
        status.ignore_rc_disconnect = false;
    } else if (dj6.right_switch == DJ6::MID) {
        // 开启自动导航，不忽略遥控器断连
        status.is_nav_on = true;
        status.ignore_rc_disconnect = false;
    } else if (dj6.right_switch == DJ6::DOWN) {
        // 开启自动导航，忽略遥控器断连
        status.is_nav_on = true;
        status.ignore_rc_disconnect = true;
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
    const float ratio1 = fabsf(nav.vel_x / settings.vxy_max);
    const float ratio2 = fabsf(nav.vel_y / settings.vxy_max);
    const float ratio = fmaxf(ratio1, ratio2);
    if (ratio > 1) {
        status.nav.vx = nav.vel_x / ratio;
        status.nav.vy = nav.vel_y / ratio;
    } else {
        status.nav.vx = nav.vel_x;
        status.nav.vy = nav.vel_y;
    }
    status.nav.rpm = 0;
}

static void notNAV() {
    // 不响应导航，自转
    status.nav.vx = 0;
    status.nav.vy = 0;
    status.nav.rpm = settings.rpm_max;
}

static void followStateMachine() {
    // 比赛尚未开始，速度置0退出
    if (referee.competition_is_started != 1) {
        status.nav.vx = 0;
        status.nav.vy = 0;
        status.nav.rpm = 0;
        return;
    }

    // 血量低且不在家里，立刻回家
    if (referee.robot_hp < settings.go_home_hp && not checkHome()) {
        status.nav_state = Status::GO_HOME;
    }

    // 状态机
    switch (status.nav_state) {
        case Status::GO_CENTER:
            status.nav_target_x = settings.center_x;
            status.nav_target_y = settings.center_y;
            followNAV();
            if (checkCenter()) {
                status.nav_state = Status::IN_CENTER;
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
                status.nav_state = Status::IN_HOME;
            }
            break;

        case Status::IN_HOME:
            notNAV();
            if (referee.robot_hp == referee.robot_hp_limit) {
                status.nav_state = Status::GO_CENTER;
            }
            break;

        default:
            break;
    }
}

static void handle_nav() {
    // 是否开启导航
    if (status.is_nav_on) {
        // 左边摇杆
        if (dj6.left_switch == DJ6::UP) {
            // 强制回家
            status.nav_target_x = settings.home_x;
            status.nav_target_y = settings.home_y;
            followNAV();
        } else if (dj6.left_switch == DJ6::MID) {
            // 强制去中心增益点
            status.nav_target_x = settings.center_x;
            status.nav_target_y = settings.center_y;
            followNAV();
        } else if (dj6.left_switch == DJ6::DOWN) {
            // 遵循状态机决策
            followStateMachine();
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
