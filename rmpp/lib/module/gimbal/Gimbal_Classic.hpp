#pragma once

#include "Gimbal.hpp"
#include "motor/Motor.hpp"

class Gimbal_Classic : public Gimbal {
public:
    // 云台电机
    struct motor_t {
        Motor &yaw, &pitch;
    } motor;

    Gimbal_Classic(const IMU& imu, const motor_t& motor);

    // 云台使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 电机角度 -> 云台姿态
    void forward() override;

    // 云台姿态 -> 电机角度
    void backward() override;
};