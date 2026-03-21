#include "Sentry.hpp"
#include "motor/GM6020.hpp"

extern GM6020 yaw2;

void Sentry::OnLoop() {
    Robot::OnLoop();

    is_game = device.referee.game.game_progress == Referee::GAMING && (device.referee.robot.id == 7 || device.referee.robot.id == 107);

    if (is_game) {
        home = GAME_HOME;
        home_r = GAME_HOME_R;
        center = GAME_CENTER;
        center_r = GAME_CENTER_R;
    } else {
        home = TEST_HOME;
        home_r = TEST_HOME_R;
        center = TEST_CENTER;
        center_r = TEST_CENTER_R;
    }

    if (is_game || device.rc.is_fn) {
        handleGame();
    } else {
        handlePause();
    }
}

bool Sentry::checkPos(const pos_t& pos, const UnitFloat<>& r) {
    const UnitFloat<m> distance = sqrt(square(pos.x - device.mavlink.current_position.x) + square(pos.y - device.mavlink.current_position.y));
    return distance <= r;
}

void Sentry::handlePause() {
    status = GO_CENTER;
    vx.software = vy.software = wr.software = 0 * default_unit;
    gimbal_mode.software = GIMBAL_SPEED_MODE;
    wyaw.software = wpitch.software = 0 * default_unit;
    yaw.software = pitch.software = 0 * default_unit;
    is_rub.software = is_shoot.software = false;
}

void Sentry::handleAutoAim() {
    // 开摩擦轮
    if (is_game) {
        is_rub.software = true;
    } else {
        is_rub.software = false;
    }

    // 自瞄
    if (device.mavlink.auto_aim.is_detect) {
        gimbal_mode.software = GIMBAL_ANGLE_MODE;
        yaw.software = device.mavlink.auto_aim.yaw;
        pitch.software = device.mavlink.auto_aim.pitch;
        dwt_auto_aim.UpdateDT();
    } else if (dwt_auto_aim.GetDT() > AUTO_AIM_TIMEOUT) { // 360°巡视
        gimbal_mode.software = GIMBAL_SPEED_MODE;
        wyaw.software = YAW_SPEED;
        static bool is_up = false;
        if (is_up) {
            wpitch.software = PITCH_SPEED;
            if (device.gimbal.pitch.imu.measure > PITCH_MAX) is_up = false;
        } else {
            wpitch.software = -PITCH_SPEED;
            if (device.gimbal.pitch.imu.measure < PITCH_MIN) is_up = true;
        }
    }

    // 自动火控
    if (is_game) {
        is_shoot.software = device.mavlink.auto_aim.is_fire;
    } else {
        is_shoot.software = false;
    }
}

void Sentry::handleGame() {
    switch (status) {
        case GO_CENTER:
            // 导航
            device.mavlink.target_position.x = center.x;
            device.mavlink.target_position.y = center.y;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 小陀螺
            wr.software = GO_WR;

            // 云台旋转
            gimbal_mode.software = GIMBAL_SPEED_MODE;
            wyaw.software = YAW_SPEED;
            wpitch.software = 0 * default_unit;

            // 检查是否到中心点
            if (checkPos(center, center_r)) {
                status = IN_CENTER;
            }
            break;

        case IN_CENTER:
            // 不响应导航
            vx.software = vy.software = 0 * default_unit;

            // 小陀螺
            wr.software = CENTER_WR;

            // 自瞄
            handleAutoAim();

            // 血量小于等于20%回家（400 * 20% = 80)
            if (device.referee.robot.hp <= 80) {
                status = GO_HOME;
            }
            break;

        case GO_HOME:
            // 导航
            device.mavlink.target_position.x = home.x;
            device.mavlink.target_position.y = home.y;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 小陀螺
            wr.software = GO_WR;

            // 云台旋转
            gimbal_mode.software = GIMBAL_SPEED_MODE;
            wyaw.software = YAW_SPEED;
            wpitch.software = 0 * default_unit;

            // 检查是否到家
            if (checkPos(home, home_r)) {
                status = IN_HOME;
            }
            break;

        case IN_HOME:
            // 不响应导航
            vx.software = vy.software = 0 * default_unit;

            // 小陀螺
            wr.software = HOME_WR;

            // 云台停止旋转
            gimbal_mode.software = GIMBAL_SPEED_MODE;
            wyaw.software = 0 * default_unit;
            wpitch.software = 0 * default_unit;

            // 血量恢复满去中心点
            if (device.referee.robot.hp >= 400) {
                status = GO_CENTER;
            }
            break;

        default:
            break;
    }
}