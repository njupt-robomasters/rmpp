#include "Gimbal_Classic.hpp"

Gimbal_Classic::Gimbal_Classic(const IMU& imu, const motor_t& motor) : Gimbal(imu), motor(motor) {}

void Gimbal_Classic::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    motor.yaw.SetEnable(is_enable);
    motor.pitch.SetEnable(is_enable);
}

void Gimbal_Classic::OnLoop() {
    // 更新云台转动
    updateMotion();

    // 角度解算
    angleBackward();
    angleForward();

    // 电机PID计算
    motor.yaw.OnLoop();
    motor.pitch.OnLoop();
}

void Gimbal_Classic::angleForward() {
    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 设置电机角度
    yaw.ecd.ref = motor.yaw.SetAngle(yaw.ecd.ref, -chassis_wr + yaw_speed); // 同时设置前馈（前馈与底盘速度方向相反，因为云台期望静止）
    pitch.ecd.ref = motor.pitch.SetAngle(pitch.ecd.ref, pitch_speed);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}

void Gimbal_Classic::angleBackward() {
    // 从电机读取
    yaw.ecd.measure = motor.yaw.angle.measure;
    pitch.ecd.measure = motor.pitch.angle.measure;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}