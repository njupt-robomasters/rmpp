#pragma once

#include "unit.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "dm4310.hpp"
#include "m6020.hpp"

extern "C" void task_can_entry(const void* argument);

class Gimbal {
    friend void task_can_entry(const void* argument);

public:
    enum mode_e {
        ECD_MODE,
        IMU_MODE
    };

private:
    static constexpr Angle<deg> YAW_OFFSET = -120.0f * deg; // yaw重合于底盘正方向时的绝对角度
    static constexpr Angle<deg> PITCH_OFFSET = 160.0f * deg; // pitch水平时的绝对角度
    static constexpr Angle<deg> PITCH_MIN = 130.0f * deg; // pitch最小绝对角度
    static constexpr Angle<deg> PITCH_MAX = -175.0f * deg; // pitch最大绝对角度

    const IMU& imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    // 电机对象
    M6020 m_yaw;
    DM4310 m_pitch;

    bool is_enable = false; // 云台使能标志
    mode_e mode = ECD_MODE; // 云台模式
    Unit<rpm> yaw_speed_ff; // yaw速度前馈（小陀螺模式需要）

    void setCurrentAsTarget();

    void forwardCalc();

    void inverseCalc();

public:
    struct {
        struct {
            Angle<deg> ref, measure;
        } absolute, relative, imu;
    } yaw, pitch;

    Gimbal(const IMU& imu, PID::param_t* yaw_pid, PID::param_t* pitch_pid);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetMode(mode_e mode);

    void SetAngle(Angle<deg> yaw, Angle<deg> pitch);

    void AddAngle(Angle<deg> pitch, Angle<deg> yaw);

    void SetYawSpeedFF(Unit<rpm> yaw_speed_ff);

    void Update();
};
