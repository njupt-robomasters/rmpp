#include "gimbal.hpp"

Gimbal::Gimbal(const IMU& imu, PID::param_t* yaw1_pid_param, PID::param_t* yaw2_pid_param, PID::param_t* pitch_pid_param) :
    imu(imu),
    m_yaw1(1, 0x01),
    m_yaw2(1, 0x205),
    m_pitch(2, 0x00, 0x01) {
    // 设置PID参数
    m_yaw1.SetPIDParam(yaw1_pid_param);
    m_yaw2.SetPIDParam(yaw2_pid_param);
    m_pitch.SetPIDParam(pitch_pid_param);

    // 反转电机
    m_pitch.SetInvert(true);
}

// 设置当前位置为目标位置
void Gimbal::setCurrentAsTarget() {
    if (mode == ECD_MODE) {
        yaw.relative.ref = yaw.relative.measure;
        pitch.relative.ref = pitch.relative.measure;
    } else if (mode == IMU_MODE) {
        yaw.imu.ref = yaw.imu.measure;
        pitch.imu.ref = pitch.imu.measure;
    }
}

void Gimbal::addAngle(const Angle<deg>& yaw, const Angle<deg>& pitch) {
    if (mode == ECD_MODE) {
        this->yaw.relative.ref = this->yaw.relative.ref + yaw;
        this->pitch.relative.ref = this->pitch.relative.ref + pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = this->yaw.imu.ref + yaw;
        this->pitch.imu.ref = this->pitch.imu.ref + pitch;
    }
}


void Gimbal::forwardCalc() {
    // 用于参考系转换
    const Angle yaw_imu_minus_relative = yaw.imu.measure - yaw.relative.measure;
    const Angle pitch_imu_minus_relative = pitch.imu.measure - pitch.relative.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.relative.ref + yaw_imu_minus_relative;
        pitch.imu.ref = pitch.relative.ref + pitch_imu_minus_relative;
    } else if (mode == IMU_MODE) {
        // 换算到relative参考系
        yaw.relative.ref = yaw.imu.ref - yaw_imu_minus_relative;
        pitch.relative.ref = pitch.imu.ref - pitch_imu_minus_relative;
    }

    // 分配大小yaw
    yaw1.relative.ref = yaw.relative.ref;
    // yaw2.relative.ref = yaw.relative.ref - yaw1.relative.measure;
    yaw2.relative.ref = 0; // 暂时锁定小yaw

    // absolute
    yaw1.absolute.ref = yaw1.relative.ref + YAW1_OFFSET;
    yaw2.absolute.ref = yaw2.relative.ref + YAW2_OFFSET;
    pitch.absolute.ref = pitch.relative.ref + PITCH_OFFSET;

    // pitch软件限位
    Angle angle_err;
    angle_err = pitch.absolute.ref - PITCH_MIN;
    if (angle_err < 0) pitch.absolute.ref = PITCH_MIN;
    angle_err = pitch.absolute.ref - PITCH_MAX;
    if (angle_err > 0) pitch.absolute.ref = PITCH_MAX;
    // 传递限位
    pitch.relative.ref = pitch.absolute.ref - PITCH_OFFSET;
    pitch.imu.ref = pitch.relative.ref + pitch_imu_minus_relative;

    // 应用到电机
    m_pitch.SetAngle(pitch.absolute.ref);
    m_yaw1.SetAngle(yaw1.absolute.ref, UnitFloat<deg_s>(chassis_vr));
    m_yaw2.SetAngle(yaw2.absolute.ref);
}

void Gimbal::backwardCalc() {
    // absolute，从电机读取
    yaw1.absolute.measure = m_yaw1.angle.measure;
    yaw2.absolute.measure = m_yaw2.angle.measure;
    pitch.absolute.measure = m_pitch.angle.measure;

    // relative
    yaw1.relative.measure = m_yaw1.angle.measure - YAW1_OFFSET;
    yaw2.relative.measure = m_yaw2.angle.measure - YAW2_OFFSET;
    yaw.relative.measure = yaw1.relative.measure + yaw2.relative.measure;
    pitch.relative.measure = m_pitch.angle.measure - PITCH_OFFSET;

    // imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}

void Gimbal::WaitReady() {
    m_yaw1.WaitReady();
    m_yaw2.WaitReady();
    m_pitch.WaitReady();
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    m_yaw1.SetEnable(is_enable);
    m_yaw2.SetEnable(is_enable);
    m_pitch.SetEnable(is_enable);

    setCurrentAsTarget();
}

void Gimbal::SetMode(const mode_e mode) {
    if (this->mode == mode) return;
    this->mode = mode;

    setCurrentAsTarget();
}

void Gimbal::SetAngle(const Angle<>& yaw, const Angle<>& pitch) {
    if (mode == ECD_MODE) {
        this->yaw.relative.ref = yaw;
        this->pitch.relative.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
}

void Gimbal::SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void Gimbal::SetChassisVR(const UnitFloat<> chassis_vr) {
    this->chassis_vr = chassis_vr;
}

void Gimbal::Update() {
    const float dt = dwt.GetDT();
    if (is_enable) {
        const Angle<deg> yaw_add = yaw_speed * dt;
        const Angle<deg> pitch_add = pitch_speed * dt;
        addAngle(yaw_add, pitch_add);
    }

    backwardCalc();
    forwardCalc();

    m_pitch.Update();
    m_yaw1.Update();
    m_yaw2.Update();
}
