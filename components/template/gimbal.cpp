#include "gimbal.hpp"

Gimbal_Template::Gimbal_Template(const IMU& imu) : imu(imu) {}

void Gimbal_Template::SetMode(const mode_e mode) {
    if (this->mode == mode) return;
    this->mode = mode;

    setCurrentAsTarget();
}

void Gimbal_Template::SetAngle(const Angle<>& yaw, const Angle<>& pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
}

void Gimbal_Template::SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void Gimbal_Template::SetChassisVR(const UnitFloat<>& chassis_vr) {
    this->chassis_vr = chassis_vr;
}

void Gimbal_Template::setCurrentAsTarget() {
    backwardCalc(); // 从电机读取数据
    if (mode == ECD_MODE) {
        yaw.ecd.ref = yaw.ecd.measure;
        pitch.ecd.ref = pitch.ecd.measure;
    } else if (mode == IMU_MODE) {
        yaw.imu.ref = yaw.imu.measure;
        pitch.imu.ref = pitch.imu.measure;
    }
}

void Gimbal_Template::updateMotion(const float dt) {
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
