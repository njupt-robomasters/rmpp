#include "Sentry.hpp"

#include "motor/GM6020.hpp"
extern GM6020 yaw2;

void Sentry::OnLoop() {
    Infantry::OnLoop();

    // 按下停止键忽略上位机指令
    if (device.rc.vt13.pause) {
        device.mavlink.is_enable = false;
    }

    // 失能释放忽略上位机指令
    if (device.rc.is_enable == false) {
        device.mavlink.is_enable = false;
    }


}

void Sentry::handleGame() {

}

void Sentry::handleTest() {
    static bool last_pause = false;

    if (device.rc.vt13.fn_left && !last_pause) {
        if ( nav_status == ORIGIN ) {
            device.mavlink.pos.pos_x = 0 *m_s;
            device.mavlink.pos.pos_y = 0 *m_s;
            nav_status = POS1;
        } else if ( nav_status == POS1) {
            device.mavlink.pos.pos_x = 1 *m_s;
            device.mavlink.pos.pos_y = 0 *m_s;
            nav_status = POS2;
        } else if ( nav_status == POS2) {
            device.mavlink.pos.pos_x = 1 *m_s;
            device.mavlink.pos.pos_y = 1 *m_s;
            nav_status = ORIGIN;
        }
    }
    last_pause = device.rc.vt13.fn_left;
}

void Sentry::handleRC() {
    device.rc.OnLoop();

    vx.rc = device.rc.x * config.vxy_max;
    vy.rc = device.rc.y * config.vxy_max;
    wr.rc = device.rc.r * config.wr_max;
    yaw_speed.rc = device.rc.yaw * config.yaw_speed_max;
    pitch_speed.rc = device.rc.pitch * config.pitch_speed_max;
    is_rub.rc = device.rc.is_rub;;
    is_shoot.rc = device.rc.is_shoot;

    //左fn键：哨兵导航调试
    if (device.mavlink.is_connect == true) {
        if (device.referee.game.progress == Referee::GAMING) {
            handleGame();
        } else {
            handleTest();
        }
    }

    // 右fn键：底盘跟随模式
    if (device.rc.vt13.fn_right) {
        device.chassis.SetMode(Chassis::FOLLOW_MODE);
    }

    // 校准yaw偏移
    static bool set_yaw_zero_last = false;
    const bool set_yaw_zero = device.rc.vt13.mode == VT13::C && device.rc.vt13.fn_left && device.rc.vt13.fn_right;
    if (set_yaw_zero && !set_yaw_zero_last) {
        device.gimbal.SetYawZero();
        device.gimbal.SaveYawOffset(device.flashdb);
        device.buzzer.Play(Buzzer::G5G5G5);
    }
    set_yaw_zero_last = set_yaw_zero;
}

void Sentry::handleChassis() {
    //旋转vx.mavlink
    auto [vx_mavlink_transformed,vy_mavlink_transformed] = rotate(device.mavlink.cmd_vel.vel_x, device.mavlink.cmd_vel.vel_y, yaw2.angle.measure);

    if (device.mavlink.is_enable == false) {
        vx_mavlink_transformed.value = 0.0;
        vy_mavlink_transformed.value = 0.0;
    }

    // 设置底盘速度
    vx.sum = unit::clamp(vx.rc + vx.client + vx_mavlink_transformed, config.vxy_max);
    vy.sum = unit::clamp(vy.rc + vy.client + vy_mavlink_transformed, config.vxy_max);
    wr.sum = unit::clamp(wr.rc + wr.client, config.wr_max);
    device.chassis.SetSpeed(vx.sum, vy.sum, wr.sum);

    // 设置前进正方向
    device.chassis.SetGimbalYaw(device.gimbal.yaw.ecd.measure);

    // 设置功率限制
    device.chassis.SetPowerLimit(device.referee.chassis.power_limit);

    // 设置缓冲能量
    device.chassis.SetBufferEnergy(device.referee.chassis.buffer_energy);

    device.chassis.OnLoop();
}