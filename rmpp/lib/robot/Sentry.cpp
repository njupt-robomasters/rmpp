#include "Sentry.hpp"
#include "motor/GM6020.hpp"

extern GM6020 yaw2;

void Sentry::OnLoop() {
    Robot::OnLoop();

    // 比赛开始，并且为哨兵
    is_game = device.referee.game.game_progress == Referee::GAMING && (device.referee.robot.id == 7 || device.referee.robot.id == 107);

    if (is_game) {
        home = GAME_HOME;
        center = GAME_CENTER;
        center_r = GAME_CENTER_R;
    } else {
        home = TEST_HOME;
        center = TEST_CENTER;
        center_r = TEST_CENTER_R;
    }

    if (is_game || device.rc.is_fn) {
        handleNav();
        handleAutoAim();
    } else {
        handlePause();
    }
}

void Sentry::handlePause() {
    // 导航状态机
    nav_status = GO_CENTER;

    // 底盘
    vx.software = vy.software = wr.software = 0 * default_unit;

    // 云台
    gimbal_mode.software = GIMBAL_SPEED_MODE;
    wyaw.software = wpitch.software = 0 * default_unit;
    yaw.software = pitch.software = 0 * default_unit;

    // 发射机构
    is_rub.software = is_shoot.software = false;
}

void Sentry::handleNav() {
    switch (nav_status) {
        case GO_CENTER: {
            // 导航
            device.mavlink.target_position.x = center.x;
            device.mavlink.target_position.y = center.y;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 小陀螺
            wr.software = GO_WR;

            // 检查是否到中心点
            // 通过导航坐标检测
            const UnitFloat<m> distance = sqrt(square(center.x - device.mavlink.current_position.x) + square(center.y - device.mavlink.current_position.y));
            if (distance <= center_r) {
                nav_status = IN_CENTER;
            }
            // 通过RFID检测
            if (device.referee.rfid.in_center) {
                nav_status = IN_CENTER;
            }
            break;
        }

        case IN_CENTER: {
            // 导航
            device.mavlink.target_position.x = center.x;
            device.mavlink.target_position.y = center.y;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 小陀螺
            wr.software = CENTER_WR;

            // 血量小于等于20%回家（400 * 20% = 80)
            if (device.referee.robot.hp <= 80) {
                nav_status = GO_HOME;
            }
            break;
        }

        case GO_HOME: {
            // 导航
            device.mavlink.target_position.x = home.x;
            device.mavlink.target_position.y = home.y;
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 小陀螺
            wr.software = GO_WR;

            // 血量恢复满去中心点
            if (device.referee.robot.hp >= 400) {
                nav_status = GO_CENTER;
            }
        }
        break;

        default:
            break;
    }
}

void Sentry::handleAutoAim() {
    // 比赛自动开摩擦轮
    if (is_game) {
        is_rub.software = true;
    }

    // 自瞄
    if (device.mavlink.auto_aim.is_detect) { // 检测到敌方机器人
        gimbal_mode.software = GIMBAL_ANGLE_MODE;
        yaw.software = device.mavlink.auto_aim.yaw;
        pitch.software = device.mavlink.auto_aim.pitch;
        dwt_auto_aim.UpdateDT();
    } else if (dwt_auto_aim.GetDT() > AUTO_AIM_TIMEOUT) { // 未检测到敌方机器人一段时间后，360°巡视
        gimbal_mode.software = GIMBAL_SPEED_MODE;
        wyaw.software = YAW_SPEED;

        // pitch上下摆动
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
    is_shoot.software = device.mavlink.auto_aim.is_fire;
}