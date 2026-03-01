#include "Sentry.hpp"

#include "motor/GM6020.hpp"
extern GM6020 yaw2;

void Sentry::OnLoop() {
    Infantry::OnLoop();

    handleDebug();
}


void Sentry::handleDebug() {
    static bool last_pause = false;

    if (device.rc.vt13.pause && !last_pause) {
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
    last_pause = device.rc.vt13.pause;
}


void Sentry::handleChassis() {

    //旋转vx.mavlink
    auto [vx_mavlink_transformed,vy_mavlink_transformed] = rotate(nav_vx.mavlink, nav_vy.mavlink, yaw2.angle.measure);

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