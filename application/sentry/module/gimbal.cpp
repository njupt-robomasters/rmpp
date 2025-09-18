#include "gimbal.hpp"

Gimbal::Gimbal(const IMU& imu, PID::param_t* yaw_pid_param, PID::param_t* pitch_pid_param) :
    imu(imu),
    m_yaw(1, 0x205),
    m_pitch(2, 0x00, 0x01) {
    m_pitch.SetInvert(true);
    m_yaw.SetPIDParam(yaw_pid_param);
    m_pitch.SetPIDParam(pitch_pid_param);
}

// 设置当前位置为目标位置
void Gimbal::setCurrentAsTarget() {
    if (mode == ECD_MODE) {
        // relative
        yaw.relative.ref = yaw.relative.measure;
        pitch.relative.ref = pitch.relative.measure;
    } else if (mode == IMU_MODE) {
        yaw.imu.ref = yaw.imu.measure;
        pitch.imu.ref = pitch.imu.measure;
    }
}

void Gimbal::WaitReady() {
    m_yaw.WaitReady();
    m_pitch.WaitReady();
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    m_yaw.SetEnable(is_enable);
    m_pitch.SetEnable(is_enable);

    setCurrentAsTarget();
}

void Gimbal::SetMode(const mode_e mode) {
    if (this->mode == mode) return;
    this->mode = mode;

    setCurrentAsTarget();
}

void Gimbal::SetAngle(const Angle<deg> yaw, const Angle<deg> pitch) {
    if (mode == ECD_MODE) {
        this->yaw.relative.ref = yaw;
        this->pitch.relative.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
}

void Gimbal::AddAngle(const Angle<deg> yaw, const Angle<deg> pitch) {
    if (mode == ECD_MODE) {
        this->yaw.relative.ref = this->yaw.relative.ref + yaw;
        this->pitch.relative.ref = this->pitch.relative.ref + pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = this->yaw.imu.ref + yaw;
        this->pitch.imu.ref = this->yaw.imu.ref + pitch;
    }
}

void Gimbal::SetYawSpeedFF(const Unit<rpm> yaw_speed_ff) {
    this->yaw_speed_ff = yaw_speed_ff;
}

void Gimbal::Update() {
    forwardCalc();
    inverseCalc();

    m_pitch.Update();
    m_yaw.Update();
}

void Gimbal::forwardCalc() {
    // absolute，从电机读取
    yaw.absolute.measure = m_yaw.angle.measure;
    pitch.absolute.measure = m_pitch.angle.measure;

    // relative
    yaw.relative.measure = m_yaw.angle.measure - YAW_OFFSET;
    pitch.relative.measure = m_pitch.angle.measure - PITCH_OFFSET;

    // imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}

void Gimbal::inverseCalc() {
    Angle yaw_imu_minus_relative = yaw.imu.measure - yaw.relative.measure;
    Angle pitch_imu_minus_relative = pitch.imu.measure - pitch.relative.measure;

    if (mode == ECD_MODE) {
        // imu
        yaw.imu.ref = yaw.relative.ref + yaw_imu_minus_relative;
        pitch.imu.ref = pitch.relative.ref + pitch_imu_minus_relative;
    } else if (mode == IMU_MODE) {
        // relative
        yaw.relative.ref = yaw.imu.ref - yaw_imu_minus_relative;
        pitch.relative.ref = pitch.imu.ref - pitch_imu_minus_relative;
    }

    // absolute
    yaw.absolute.ref = yaw.relative.ref + YAW_OFFSET;
    pitch.absolute.ref = pitch.relative.ref + PITCH_OFFSET;

    // pitch软件限位
    Angle angle_err = pitch.absolute.ref - PITCH_MIN;
    if (angle_err < 0) pitch.absolute.ref = PITCH_MIN;
    angle_err = pitch.absolute.ref - PITCH_MAX;
    if (angle_err > 0) pitch.absolute.ref = PITCH_MAX;
    // 传递限位
    pitch.relative.ref = pitch.absolute.ref - PITCH_OFFSET;
    pitch.imu.ref = pitch.relative.ref + pitch_imu_minus_relative;

    // 应用到电机
    m_pitch.SetAngle(pitch.absolute.ref);
    m_yaw.SetAngle(yaw.absolute.ref);
}
