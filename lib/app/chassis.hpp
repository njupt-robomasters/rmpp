#pragma once

#include "unit/include_me.hpp"
#include "controller/pid.hpp"

class Chassis_Template {
public:
    enum mode_e {
        DETACH_MODE,
        FOLLOW_MODE
    };

    // 底盘使能标志
    bool is_enable = false;

    // 底盘速度
    // vx 前后速度，前为正
    // vy 左右速度，左为正
    struct {
        struct {
            UnitFloat<m_s> ref, measure;
        } chassis, gimbal;
    } vx, vy;

    // vz 旋转线速度，逆时针为正
    struct {
        UnitFloat<m_s> ref, measure;
    } vz;

    // vr 旋转角速度，逆时针为正
    struct {
        struct {
            UnitFloat<rpm> input, follow, sum;
        } ref;
        UnitFloat<rpm> measure;
    } vr;

    // 用于底盘功率控制
    UnitFloat<W> power_limit = 120 * W;       // 功率限制
    UnitFloat<W> power_estimate;              // 当前功率估计
    UnitFloat<pct> current_ratio = 100 * pct; // 电流衰减系数

    // 底盘使能/失能
    virtual void SetEnable(bool is_enable);

    void SetMode(mode_e mode);

    // 设置速度
    void SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr);

    // 设置前进正方向（云台yaw角度）
    void SetGimbalYaw(const Angle<>& gimbal_yaw);

    // 设置功率限制
    void SetPowerLimit(const UnitFloat<>& power);

    // 需要在循环中调用
    virtual void OnLoop();

protected:
    mode_e mode = DETACH_MODE;

    // 前进正方向（云台相对于底盘的yaw）
    Angle<deg> gimbal_yaw;

    // 底盘跟随PID
    PID follow_pid;

    void calcFollow();

    // 车体速度 -> 轮子速度
    virtual void speedForward() = 0;

    // 轮子速度 -> 车体速度
    virtual void speedBackward() = 0;

    // 功率控制
    virtual void powerControl() = 0;
};