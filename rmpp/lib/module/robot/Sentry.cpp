#include "Sentry.hpp"
#include "motor/GM6020.hpp"

extern GM6020 yaw2;

void Sentry::OnLoop() {
    Infantry::OnLoop();

    // 暂停
    if (device.rc.vt13.pause) {
        is_pause = true;
    }

    // 失能复位暂停
    if (device.rc.is_enable == false) {
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

void Sentry::handlePause() {
    vx.software = vy.software = wr.software = 0 * default_unit;
    gimbal_mode.software = GIMBAL_SPEED_MODE;
    wyaw.software = wpitch.software = 0 * default_unit;
    is_rub.software = is_shoot.software = false;
}

void Sentry::handleGame() {}

void Sentry::handleTest() {
    static bool fn_left_last = false;
    if (device.rc.vt13.fn_left && !fn_left_last) {
        if (nav_status == ORIGIN) {
            device.mavlink.target_position.x = 0 * m_s;
            device.mavlink.target_position.y = 0 * m_s;
            nav_status = POS1;
        } else if (nav_status == POS1) {
            device.mavlink.target_position.x = 1 * m_s;
            device.mavlink.target_position.y = 0 * m_s;
            nav_status = POS2;
        } else if (nav_status == POS2) {
            device.mavlink.target_position.x = 1 * m_s;
            device.mavlink.target_position.y = 1 * m_s;
            nav_status = ORIGIN;
        }
    }
    fn_left_last = device.rc.vt13.fn_left;

    std::tie(vx.software, vy.software) = rotate(device.mavlink.chassis_speed.vx, device.mavlink.chassis_speed.vy, yaw2.angle.measure);
}