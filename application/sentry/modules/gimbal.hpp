#pragma once

#include "unit.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "mf9025.hpp"
#include "m6020.hpp"
#include "dm4310.hpp"

extern "C" void task_can_entry(const void* argument);

class Gimbal {
    friend void task_can_entry(const void* argument);

public:
    enum mode_e {
        ECD_MODE,
        IMU_MODE
    };

private:
    // 电机偏移参数
    // 大yaw
    static constexpr Angle<deg> YAW1_OFFSET = -54.0f * deg; // 大yaw与前进方向重合时的绝对角度
    // 小yaw
    static constexpr Angle<deg> YAW2_OFFSET = -120.0f * deg; // 小yaw相对于大yaw居中时的绝对角度
    static constexpr Angle<deg> YAW2_MIN = 150.0f * deg; // 小yaw最右角度
    static constexpr Angle<deg> YAW2_MAX = -30.0f * deg; // 小yaw最左角度
    // pitch
    static constexpr Angle<deg> PITCH_OFFSET = 160.0f * deg; // pitch水平时的绝对角度
    static constexpr Angle<deg> PITCH_MIN = 130.0f * deg; // pitch最小绝对角度
    static constexpr Angle<deg> PITCH_MAX = -175.0f * deg; // pitch最大绝对角度

    const IMU& imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    // 电机对象
    MF9025 m_yaw1;
    M6020 m_yaw2;
    DM4310 m_pitch;

    bool is_enable = false; // 云台使能标志
    mode_e mode = IMU_MODE; // 云台模式
    UnitFloat<rpm> chassis_vr; // yaw速度前馈（小陀螺模式需要）

    BSP::Dwt dwt; // 维护dt

    void setCurrentAsTarget();

    void addAngle(const Angle<>& yaw, const Angle<>& pitch);

    void forwardCalc();

    void backwardCalc();

public:
    // 总yaw
    struct {
        struct {
            Angle<deg> ref, measure;
        } relative, imu;
    } yaw;

    // 大小yaw分配
    struct {
        struct {
            Angle<deg> ref, measure;
        } absolute, relative;
    } yaw1, yaw2;

    // pitch
    struct {
        struct {
            Angle<deg> ref, measure;
        } absolute, relative;

        struct {
            Angle<deg> ref, measure;
        } imu;
    } pitch;

    // 云台运动速度
    UnitFloat<deg_s> yaw_speed, pitch_speed;

    Gimbal(const IMU& imu, PID::param_t* yaw1_pid_param, PID::param_t* yaw2_pid_param, PID::param_t* pitch_pid_param);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetMode(mode_e mode);

    void SetAngle(const Angle<>& yaw, const Angle<>& pitch);

    void SetSpeed(const UnitFloat<>& yaw_speed, const UnitFloat<>& pitch_speed);

    void SetChassisVR(const UnitFloat<>& chassis_vr);

    void Update();
};
