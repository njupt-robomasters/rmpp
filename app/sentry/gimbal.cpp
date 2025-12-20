#include "gimbal.hpp"

Gimbal::Gimbal(const IMU& imu, PID::param_t* yaw1_pid, PID::param_t* yaw2_pid, PID::param_t* pitch_pid) :
    Gimbal_Template(imu),
    m_yaw1(1, 1),
    m_yaw2(1, 1),
    m_pitch(2, 0x00, 0x01) {
    // 设置电机PID参数
    m_yaw1.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, yaw1_pid);  // 大yaw
    m_yaw2.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, yaw2_pid);  // 小yaw
    m_pitch.SetPID(Motor::ANGLE_MODE, Motor::TORQUE_TYPE, pitch_pid); // pitch

    // 设置电机正方向
    m_yaw1.SetInvert(false); // 大yaw
    m_yaw2.SetInvert(false); // 小yaw
    m_pitch.SetInvert(true); // pitch

    // 设置电机偏移
    m_yaw1.SetOffset(YAW1_OFFSET);   // 大yaw
    m_yaw2.SetOffset(YAW2_OFFSET);   // 小yaw
    m_pitch.SetOffset(PITCH_OFFSET); // pitch

    // 设置电机限位
    m_yaw1.SetLimit(false);                       // 大yaw
    m_yaw2.SetLimit(true, YAW2_MIN, YAW2_MAX);    // 小yaw
    m_pitch.SetLimit(true, PITCH_MIN, PITCH_MAX); // pitch
}

void Gimbal::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 设置当前位置为目标位置
    setCurrentAsTarget();

    m_yaw1.SetEnable(is_enable);
    m_yaw2.SetEnable(is_enable);
    m_pitch.SetEnable(is_enable);
}

void Gimbal::OnLoop() {
    // 更新云台转动
    updateMotion();

    // 角度解算
    angleBackward();
    angleForward();

    // 电机PID计算
    m_pitch.OnLoop();
    m_yaw1.OnLoop();
    m_yaw2.OnLoop();
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

    // 分配大小yaw
    yaw1.ref = yaw.ecd.ref;                // 大yaw直接设置为目标值
    yaw2.ref = yaw.ecd.ref - yaw1.measure; // 大yaw未转到目标，小yaw来补偿

    // 设置电机角度
    m_yaw1.SetAngle(yaw1.ref, -chassis_wr);
    m_yaw2.SetAngle(yaw2.ref,  -m_yaw1.speed.measure - chassis_wr);
    m_pitch.SetAngle(pitch.ecd.ref);

    // 传递pitch软件限位
    pitch.ecd.ref = m_pitch.angle.ref;
    // 传递到imu参考系
    pitch.imu.ref = pitch.ecd.ref + pitch_imu_minus_ecd;
}

void Gimbal::angleBackward() {
    // 从电机读取
    yaw1.measure = m_yaw1.angle.measure;
    yaw2.measure = m_yaw2.angle.measure;
    pitch.ecd.measure = m_pitch.angle.measure;

    // 合并大小yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}