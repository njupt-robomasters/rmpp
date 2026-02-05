#pragma once

#include "imu/IMU.hpp"

class Gimbal {
public:
    bool is_enable = false; // 云台使能标志

    // 云台模式
    enum mode_e {
        ECD_MODE, // 编码器模式
        IMU_MODE, // 陀螺仪模式
    } mode = IMU_MODE;

    // 云台角度
    struct {
        struct {
            Angle<deg> ref, measure;
        } ecd, imu;

        Angle<deg> imu_minus_ecd;
    } yaw, pitch;

    UnitFloat<deg_s> yaw_speed, pitch_speed; // 云台速度

    Gimbal(const IMU& imu);

    // 云台使能/失能
    virtual void SetEnable(bool is_enable);

    // 设置云台模式：编码器模式 / IMU模式
    void SetMode(mode_e mode);

    // 设置速度
    void SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed);

    // 设置角度
    void SetAngle(const Angle<>& yaw, const Angle<>& pitch);

    // 设置底盘旋转速度，用于yaw轴前馈
    void SetChassisWr(const UnitFloat<>& chassis_wr);

    // 需要在循环中调用
    virtual void OnLoop();

protected:
    const IMU& imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    UnitFloat<rpm> chassis_wr; // yaw速度前馈（小陀螺模式需要）

    BSP::Dwt dwt_motion; // 用于云台运动

    // 处理云台转动，通过dt不断计算角度增量
    void handleMotion();

    // 电机角度 -> 云台姿态
    virtual void forward() = 0;

    // 云台姿态 -> 电机角度
    virtual void backward() = 0;
};