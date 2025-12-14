#include "control.hpp"

void Control::OnLoop() {
    // 控制器
    handle_disconnect();
    handle_dj6();
    handle_vt13();
    handle_nuc();
    handle_referee();

    // 传感器
    handle_imu();

    // 执行器
    handle_chassis();
    handle_gimbal();
    handle_shooter();
}

void Control::setEnable(const bool is_enable) {
    chassis.SetEnable(is_enable);
    gimbal.SetEnable(is_enable);
    shooter.SetEnable(is_enable);
}

void Control::handle_disconnect() {
    // 检查遥控器断联
    if (vt13.is_connected) {
        switch (vt13.mode) {
            case VT13::C:
                setEnable(false);
                break;
            case VT13::N:
                setEnable(true);
                break;
            case VT13::S:
                setEnable(true);
                break;
            case VT13::ERR:
                setEnable(false);
                break;
        }
    } else if (dj6.is_connected) {
        setEnable(true);
    } else {
        setEnable(false);
    }
}

void Control::handle_dj6() {
    vx.rc = dj6.x * speed.vxy_max;
    vy.rc = dj6.y * speed.vxy_max;
    yaw_speed.rc = dj6.yaw * speed.yaw_max;
    pitch_speed.rc = dj6.pitch * speed.pitch_max;

    // 左拨杆：发射结构
    switch (dj6.switch_left) {
        case DJ6::ERR:
            shooter.SetPrepareShoot(false);
            shooter.SetShoot(false);
            break;
        case DJ6::UP:
            shooter.SetPrepareShoot(false);
            shooter.SetShoot(false);
            break;
        case DJ6::MID:
            shooter.SetPrepareShoot(true);
            shooter.SetShoot(false);
            break;
        case DJ6::DOWN:
            shooter.SetPrepareShoot(true);
            shooter.SetShoot(true);
            break;
    }

    // 右拨杆：云台模式、测试小陀螺
    switch (dj6.switch_right) {
        case DJ6::ERR:
            vr.rc = 0 * default_unit;
            break;
        case DJ6::UP:
            gimbal.SetMode(Gimbal_Template::ECD_MODE);
            vr.rc = 0 * default_unit;
            break;
        case DJ6::MID:
            gimbal.SetMode(Gimbal_Template::IMU_MODE);
            vr.rc = 0 * default_unit;
            break;
        case DJ6::DOWN:
            gimbal.SetMode(Gimbal_Template::IMU_MODE);
            vr.rc = speed.vr_max;
            break;
    }

    dj6.OnLoop();
}

void Control::handle_vt13() {
    // 遥控器
    vx.vt13 = vt13.x * speed.vxy_max;
    vy.vt13 = vt13.y * speed.vxy_max;
    vr.vt13 = -vt13.wheel * speed.vr_max;
    yaw_speed.vt13 = vt13.yaw * speed.yaw_max;
    pitch_speed.vt13 = vt13.pitch * speed.pitch_max;

    // 客户端键鼠
    yaw_speed.client = -vt13.mouse_yaw * speed.yaw_max;
    pitch_speed.client = -vt13.mouse_pitch * speed.pitch_max;

    vt13.OnLoop();
}

void Control::handle_referee() {
    referee.OnLoop();
}

void Control::handle_nuc() {
    nuc.OnLoop();
}

void Control::handle_imu() {
    imu.OnLoop();
}

void Control::handle_chassis() {
    // 设置底盘速度
    vx.sum = unit::clamp(vx.rc + vx.vt13 + vx.client + vx.nav, speed.vxy_max);
    vy.sum = unit::clamp(vy.rc + vy.vt13 + vy.client + vy.nav, speed.vxy_max);
    vr.sum = unit::clamp(vr.rc + vr.vt13 + vr.client + vr.nav, speed.vr_max);
    chassis.SetSpeed(vx.sum, vy.sum, vr.sum);

    // 设置云台方向
    chassis.SetGimbalYaw(gimbal.yaw.ecd.measure);

    chassis.OnLoop();
}

void Control::handle_gimbal() {
    // 设置云台速度
    pitch_speed.sum = unit::clamp(pitch_speed.rc + pitch_speed.vt13 + pitch_speed.client + pitch_speed.nav, speed.pitch_max);
    yaw_speed.sum = unit::clamp(yaw_speed.rc + yaw_speed.vt13 + yaw_speed.client + yaw_speed.nav, speed.yaw_max);
    gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

    // 设置小陀螺前馈
    gimbal.SetChassisVR(chassis.vr.measure);

    gimbal.OnLoop();
}

void Control::handle_shooter() {
    shooter.SetBulletSpeed(18.0f * m_s); // 设置弹速
    shooter.SetShootFreq(5.0f * Hz);     // 设置弹频

    shooter.OnLoop();
}