#pragma once

#include "app/gimbal.hpp"
#include "motor/gm6020.hpp"
#include "motor/dm4310.hpp"

class Gimbal : public Gimbal_Template {
public:
    // 电机偏移参数
    // yaw
    static constexpr Angle YAW_OFFSET = 93.38f * deg; // yaw与前进方向重合时的绝对角度
    // pitch
    static constexpr Angle PITCH_OFFSET = -65.0f * deg;                      // pitch水平时的绝对角度
    static constexpr Angle PITCH_MIN = (-75.0f + 5.0f) * deg - PITCH_OFFSET; // pitch最低相对角度
    static constexpr Angle PITCH_MAX = (-40.0f - 5.0f) * deg - PITCH_OFFSET; // pitch最高相对角度

    // 电机对象
    GM6020 m_yaw;
    DM4310 m_pitch;

    Gimbal(const IMU& imu, PID::param_t* yaw_pid, PID::param_t* pitch_pid);

    // 云台使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 云台姿态 -> 电机角度
    void forwardCalc() override;

    // 电机角度 -> 云台姿态
    void backwardCalc() override;
};