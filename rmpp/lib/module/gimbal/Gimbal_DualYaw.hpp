#pragma once

#include "Gimbal.hpp"
#include "motor/Motor.hpp"

class Gimbal_DualYaw : public Gimbal {
public:
    // 云台电机
    struct motor_t {
        Motor &yaw1, &yaw2, &pitch; // yaw1-大yaw，yaw2-小yaw
    } motor;

    // 大小yaw分配
    struct {
        Angle<deg> ref, measure;
    } yaw1, yaw2; // yaw1-大yaw，yaw2-小yaw

    Gimbal_DualYaw(const IMU& imu, const motor_t& motor);

    // 云台使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度正解：云台姿态 -> 电机角度
    void backward() override;
};