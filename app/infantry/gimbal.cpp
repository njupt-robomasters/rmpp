#include "gimbal.hpp"

Gimbal::Gimbal(const IMU& imu, PID::param_t* yaw_pid_param, PID::param_t* pitch_pid_param) :
    Gimbal_Template(imu),
    m_yaw(1, 1),
    m_pitch(2, 0x11, 0x10) {
    // 设置电机PID参数
    m_yaw.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, yaw_pid_param);  // yaw
    m_pitch.SetPID(Motor::ANGLE_MODE, Motor::TORQUE_TYPE, pitch_pid_param); // pitch

    // 设置电机偏移
    m_yaw.SetOffset(YAW_OFFSET);   // 大yaw
    m_pitch.SetOffset(PITCH_OFFSET); // pitch

    // 设置电机正方向
    m_yaw.SetInvert(false); // 大yaw
    m_pitch.SetInvert(true); // pitch

    // 设置电机限位
    m_yaw.SetLimit(false);                        // yaw，无限位
    m_pitch.SetLimit(false, PITCH_MIN, PITCH_MAX); // pitch
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    m_yaw.SetEnable(is_enable);
    m_pitch.SetEnable(is_enable);
}

void Gimbal::OnLoop() {
    const float dt = dwt.UpdateDT();

    if (is_enable) {
        updateMotion(dt);
    }

    backwardCalc();
    forwardCalc();

    m_pitch.OnLoop();
    m_yaw.OnLoop();

}

void Gimbal::forwardCalc() {
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
    // 分配大小yaw
    yaw_6020.ref = yaw.ecd.ref;                // yaw直接设置为目标值
    // 设置电机角度
    m_yaw.SetAngle(yaw_6020.ref, -chassis_vr); // 同时设置前馈（前馈与底盘速度方向相反，因为云台期望静止）
    m_pitch.SetAngle(pitch.ecd.ref);        // pitch有限位，如果触发限位，函数内部会修改传入的角度
    // 传递软件限位
    yaw.ecd.ref = yaw_6020.ref; // 小yaw达到限位后，会传递到总yaw，阻止总yaw角度进一步增大
    // 传递到imu参考系
    yaw.imu.ref = yaw.ecd.ref + yaw_imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch_imu_minus_ecd;
}

// 电机角度 -> 云台姿态
void Gimbal::backwardCalc() {
    // 从电机读取
    yaw_6020.measure = m_yaw.angle.measure;
    pitch.ecd.measure = m_pitch.angle.measure;
    // 合并yaw
    yaw.ecd.measure = yaw_6020.measure;
    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}