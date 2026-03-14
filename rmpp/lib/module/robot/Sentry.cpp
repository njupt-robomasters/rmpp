#include "Sentry.hpp"
#include "motor/GM6020.hpp"

extern GM6020 yaw2;

void Sentry::OnLoop() {
    Robot::OnLoop();

    // 暂停
    if (device.rc.vt13.pause) {
        is_pause = true;
    }

    // 失能复位暂停（断联除外）
    if (device.rc.vt13.is_connect == true && device.rc.is_enable == false) {
        is_pause = false;
    }

    if (is_pause) {
        handlePause();
    } else {
        if (device.referee.game.game_progress == Referee::GAMING) {
            handleGame();
        } else {
            handleTest();
        }
    }
}

bool Sentry::checkPos(const pos_t& pos, const UnitFloat<>& r) {
    const UnitFloat<m> distance = sqrt(square(pos.x - device.mavlink.current_position.x) + square(pos.y - device.mavlink.current_position.y));
    return distance <= r;
}

void Sentry::handlePause() {
    vx.software = vy.software = wr.software = 0 * default_unit;
    gimbal_mode.software = GIMBAL_SPEED_MODE;
    wyaw.software = wpitch.software = 0 * default_unit;
    is_rub.software = is_shoot.software = false;
}

void Sentry::handleGame() {
    switch (game_status) {
        case GO_CENTER:
            // 导航目标点
            device.mavlink.target_position.x = CENTER.x;
            device.mavlink.target_position.y = CENTER.y;

            // 响应导航
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 关闭小陀螺
            wr.software = 0 * default_unit;

            // 检查是否到达
            if (checkPos(CENTER, CENTER_R)) {
                game_status = IN_CENTER;
            }
            break;

        case IN_CENTER:
            // 不响应导航
            vx.software = vy.software = 0 * default_unit;

            // 开启小陀螺
            wr.software = config.wr_max;

            // 等待血量小于等于20%（400 * 20% = 80)
            if (device.referee.hp <= 80) {
                game_status = GO_HOME;
            }
            break;

        case GO_HOME:
            // 导航目标点
            device.mavlink.target_position.x = HOME.x;
            device.mavlink.target_position.y = HOME.y;

            // 响应导航
            std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);

            // 关闭小陀螺
            wr.software = 0 * default_unit;

            // 检查是否到达
            if (checkPos(HOME, HOME_R)) {
                game_status = IN_HOME;
            }
            break;

        case IN_HOME:
            // 不响应导航
            vx.software = vy.software = 0 * default_unit;

            // 关闭小陀螺
            wr.software = 0 * default_unit;

            // 等待血量恢复满
            if (device.referee.hp >= 400) {
                game_status = GO_CENTER;
            }
            break;

        default:
            break;
    }
}

void Sentry::handleTest() {
    static bool fn_left_last = false;
    if (device.rc.vt13.fn_left && !fn_left_last) {
        if (test_status == GO_ORIGIN) {
            device.mavlink.target_position.x = 0 * m_s;
            device.mavlink.target_position.y = 0 * m_s;
            test_status = GO_POS1;
        } else if (test_status == GO_POS1) {
            device.mavlink.target_position.x = 1 * m_s;
            device.mavlink.target_position.y = 0 * m_s;
            test_status = GO_POS2;
        } else if (test_status == GO_POS2) {
            device.mavlink.target_position.x = 1 * m_s;
            device.mavlink.target_position.y = 1 * m_s;
            test_status = GO_ORIGIN;
        }
    }
    fn_left_last = device.rc.vt13.fn_left;

    std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);
}