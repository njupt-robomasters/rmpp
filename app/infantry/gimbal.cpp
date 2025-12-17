#include "gimbal.hpp"

Gimbal::Gimbal(const IMU& imu, PID::param_t* yaw_pid, PID::param_t* pitch_pid) :
    Gimbal_Template(imu),
    m_yaw(1, 1),
    m_pitch(2, 0x10, 0x11) {
    // 设置电机PID参数
    m_yaw.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, yaw_pid);    // yaw
    m_pitch.SetPID(Motor::ANGLE_MODE, Motor::TORQUE_TYPE, pitch_pid); // pitch

    // 设置电机正方向
    m_yaw.SetInvert(false);   // yaw
    m_pitch.SetInvert(false); // pitch

    // 设置电机偏移
    m_yaw.SetOffset(YAW_OFFSET);     // yaw
    m_pitch.SetOffset(PITCH_OFFSET); // pitch

    // 设置电机限位
    m_yaw.SetLimit(false);                        // yaw
    m_pitch.SetLimit(true, PITCH_MIN, PITCH_MAX); // pitch
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    m_yaw.SetEnable(is_enable);
    m_pitch.SetEnable(is_enable);

    // 设置当前位置位目标位置
    yaw.ecd.ref = yaw.ecd.measure;
    pitch.ecd.ref = pitch.ecd.measure;
}

void Gimbal::OnLoop() {
    updateMotion();

    angleBackward();
    angleForward();

    m_yaw.OnLoop();
    m_pitch.OnLoop();
}

void Gimbal::angleForward() {
    // 用于参考系转换
    const Angle yaw_imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    const Angle pitch_imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw_imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch_imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw_imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch_imu_minus_ecd;
    }

    // 设置电机角度
    m_yaw.SetAngle(yaw.ecd.ref, -chassis_vr); // 同时设置前馈（前馈与底盘速度方向相反，因为云台期望静止）
    m_pitch.SetAngle(pitch.ecd.ref);

    // 传递软件限位
    yaw.ecd.ref = m_yaw.angle.ref;
    pitch.ecd.ref = m_pitch.angle.ref;
    yaw.imu.ref = yaw.ecd.ref + yaw_imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch_imu_minus_ecd;
}

void Gimbal::angleBackward() {
    // 从电机读取
    yaw.ecd.measure = m_yaw.angle.measure;
    pitch.ecd.measure = m_pitch.angle.measure;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}