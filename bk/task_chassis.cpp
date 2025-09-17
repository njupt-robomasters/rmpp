#include "task_chassis.h"
#include "app.hpp"

static uint32_t dwt_cnt;
static float dt;

static DJ6::switch_e last_right_switch = DJ6::ERR;
static DJ6::switch_e last_left_switch = DJ6::ERR;
static uint32_t last_competition_is_started = 0;

static float nonlinear_transform(const float val, const float max_val, const float rate) {
    float result = val / max_val;
    if (result > 0) {
        result = powf(result, rate);
    } else {
        result = -powf(-result, rate);
    }
    result = result * max_val;
    return result;
}

static void handle_rc() {
    // 解析遥控器控制（实时状态响应）
    if (dj6.is_connected) {
        status.rc.vx = dj6.x * settings.vxy_max;
        status.rc.vy = dj6.y * settings.vxy_max;
        status.rc.rpm = nonlinear_transform(-dj6.yaw * settings.rpm_max, settings.rpm_max, 4.0f);
    } else {
        status.rc.vx = 0;
        status.rc.vy = 0;
        status.rc.rpm = 0;
    }

    // 解析右边摇杆（变化量响应）
    if (dj6.right_switch != last_right_switch) {
        last_right_switch = dj6.right_switch;
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

    // 解析左边摇杆（变化量响应）
    if (dj6.left_switch != last_left_switch) {
        last_left_switch = dj6.left_switch;
        if (dj6.left_switch == DJ6::UP) {
            // 强制回家
            status.nav_policy = Status::FORCE_HOME;
        } else if (dj6.left_switch == DJ6::MID) {
            // 强制去中心增益点
            status.nav_policy = Status::FORCE_CENTER;
        } else if (dj6.left_switch == DJ6::DOWN) {
            // 遵循状态机决策
            status.nav_policy = Status::FOLLOW_STATEMACHINE;
        }
    }
}

void static handle_game_start() {
    if (referee.competition_is_started != last_competition_is_started) {
        last_competition_is_started = referee.competition_is_started;
        if (referee.competition_is_started == 1) {
            // 比赛开始
            status.is_nav_on = true;
            status.nav_policy = Status::FOLLOW_STATEMACHINE;
            status.ignore_rc_disconnect = true;
        } else {
            // 比赛结束
            status.is_nav_on = false;
            status.ignore_rc_disconnect = false;
        }
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
    if (status.is_nav_on) {
        // 左边摇杆
        if (status.nav_policy == Status::FORCE_HOME) {
            // 强制回家
            status.nav_target_x = settings.home_x;
            status.nav_target_y = settings.home_y;
            followNAV();
        } else if (status.nav_policy == Status::FORCE_CENTER) {
            // 强制去中心增益点
            status.nav_target_x = settings.center_x;
            status.nav_target_y = settings.center_y;
            followNAV();
        } else if (status.nav_policy == Status::FOLLOW_STATEMACHINE) {
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
        dt = BSP_DWT_GetDT(&dwt_cnt);

        handle_rc();
        handle_game_start();
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
