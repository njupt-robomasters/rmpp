#pragma once

#include "imu/imu.hpp"

class Gimbal_Template {
public:
    // 云台模式
    enum mode_e {
        ECD_MODE,
        IMU_MODE
    } mode = IMU_MODE;

    // 云台使能标志
    bool is_enable = false;

    // yaw速度前馈（小陀螺模式需要）
    UnitFloat<rpm> chassis_vr;

    // 云台角度
    struct {
        struct {
            Angle<deg> ref, measure;
        } ecd, imu;
    } yaw, pitch;

    // 云台运动速度
    UnitFloat<deg_s> yaw_speed, pitch_speed;

    BSP::Dwt dwt; // 维护dt

    Gimbal_Template(const IMU& imu);

    // 设置云台模式：编码器模式 / IMU模式
    void SetMode(mode_e mode);

    // 设置角度
    void SetAngle(const Angle<>& yaw, const Angle<>& pitch);

    // 设置速度
    void SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed);

    // 设置底盘旋转速度，用于yaw轴前馈
    void SetChassisVR(const UnitFloat<>& chassis_vr);

    // 云台使能/失能
    virtual void SetEnable(bool is_enable) = 0;

    // 需要在循环中调用
    virtual void OnLoop() = 0;

protected:
    // 对陀螺仪的引用，用于云台IMU闭环模式
    const IMU& imu;

    // 更新云台转动，通过dt不断计算角度增量
    void updateMotion(float dt);

    // 云台姿态 -> 电机角度
    virtual void forwardCalc() = 0;

    // 电机角度 -> 云台姿态
    virtual void backwardCalc() = 0;
};
