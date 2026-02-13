#include "Gimbal.hpp"

Gimbal::Gimbal(const IMU& imu) : imu(imu) {}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 这里设置电机
}

void Gimbal::SetMode(const mode_e mode) {
    this->mode = mode;
}

void Gimbal::SetAngle(const Angle<>& yaw, const Angle<>& pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
}

void Gimbal::SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void Gimbal::SetChassisWr(const UnitFloat<>& chassis_wr) {
    this->chassis_wr = chassis_wr;
}

void Gimbal::OnLoop() {
    // 更新云台转动
    handleMotion();

    // 角度正逆解
    forward();
    backward();

    // 这里更新电机
}

void Gimbal::handleMotion() {
    const UnitFloat dt = dwt_motion.UpdateDT();
    if (is_enable) {
        const Angle yaw_delta = yaw_speed * dt;
        const Angle pitch_delta = pitch_speed * dt;
        if (mode == ECD_MODE) {
            yaw.ecd.ref += yaw_delta;
            pitch.ecd.ref += pitch_delta;
        } else if (mode == IMU_MODE) {
            yaw.imu.ref += yaw_delta;
            pitch.imu.ref += pitch_delta;
        }
    }
}