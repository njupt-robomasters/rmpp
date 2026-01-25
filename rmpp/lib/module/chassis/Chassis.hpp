#pragma once

#include "unit/include_me.hpp"
#include "algorithm/PID.hpp"

class Chassis {
public:
    // 底盘配置
    struct config_t {
        UnitFloat<> chassis_radius; // 底盘半径
        UnitFloat<> wheel_radius;   // 轮半径

        const PID::config_t* follow_pid_config = nullptr; // 底盘跟随PID参数
    } config;

    bool is_enable = false; // 底盘使能标志

    // 底盘模式
    enum mode_e {
        DETACH_MODE, // 底盘分离模式
        FOLLOW_MODE, // 底盘跟随模式
    } mode = DETACH_MODE;

    // 底盘速度
    // vx 前后速度，前为正
    // vy 左右速度，左为正
    struct {
        struct {
            UnitFloat<m_s> ref, measure;
        } chassis, gimbal;
    } vx, vy;

    // 旋转线速度，逆时针为正
    struct {
        UnitFloat<m_s> ref, measure;
    } vz;

    // 旋转角速度，逆时针为正
    struct {
        UnitFloat<rpm> ref, measure, input, follow;
    } wr;

    // 用于底盘功率控制
    struct {
        UnitFloat<W> limit = 120 * W;     // 功率限制
        UnitFloat<W> estimate;            // 当前功率估计
        UnitFloat<pct> ratio = 100 * pct; // 电流/力矩衰减系数
    } power;

    Chassis(const config_t& config);

    // 底盘使能/失能
    virtual void SetEnable(bool is_enable);

    // 设置底盘跟随/分离模式
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
    Angle<deg> gimbal_yaw; // 前进正方向（云台相对于底盘的yaw）

    PID follow_pid; // 底盘跟随PID

    void calcFollow();

    // 车体速度 -> 轮子速度
    virtual void speedForward() = 0;

    // 轮子速度 -> 车体速度
    virtual void speedBackward() = 0;

    // 功率控制
    virtual void powerControl() = 0;
};