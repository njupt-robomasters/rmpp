#pragma once

#include "template/gimbal.hpp"
#include "motor/mf9025.hpp"
#include "motor/gm6020.hpp"
#include "motor/dm4310.hpp"

class Gimbal : public Gimbal_Template {
public:
    // 电机偏移参数
    // 大yaw
    static constexpr Angle YAW1_OFFSET = -3.0f * deg; // 大yaw与前进方向重合时的绝对角度
    // 小yaw
    static constexpr Angle YAW2_OFFSET = -120.0f * deg; // 小yaw相对于大yaw居中时的绝对角度
    static constexpr Angle YAW2_MIN = -120.0f * deg;    // 小yaw最右相对角度
    static constexpr Angle YAW2_MAX = 120.0f * deg;     // 小yaw最左相对角度
    // pitch
    static constexpr Angle PITCH_OFFSET = 160.0f * deg; // pitch水平时的绝对角度
    static constexpr Angle PITCH_MIN = -30.0f * deg;    // pitch最低相对角度
    static constexpr Angle PITCH_MAX = 25.0f * deg;     // pitch最高相对角度

    // 大小yaw分配
    struct {
        Angle<deg> ref, measure;
    } yaw1, yaw2; // yaw1为大yaw，yaw2为小yaw

    // 电机对象
    MF9025 m_yaw1;
    GM6020 m_yaw2;
    DM4310 m_pitch;

    Gimbal(const IMU& imu, PID::param_t* yaw1_pid_param, PID::param_t* yaw2_pid_param, PID::param_t* pitch_pid_param);

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
