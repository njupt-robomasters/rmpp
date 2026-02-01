#pragma once

#include "unit/include_me.hpp"
#include "algorithm/PID.hpp"

class Chassis {
public:
    // 底盘配置
    struct config_t {
        UnitFloat<> chassis_radius; // 底盘半径
        UnitFloat<> wheel_radius;   // 轮半径
        const PID::config_t* vxyz_pid_config = nullptr;
        const PID::config_t* follow_pid_config = nullptr; // 底盘跟随PID参数
    } config;

    // 底盘模式
    enum mode_e {
        DETACH_MODE, // 底盘分离模式
        FOLLOW_MODE, // 底盘跟随模式
    };

    // 底盘速度
    // vx 前后速度，前为正
    // vy 左右速度，左为正
    struct {
        struct vxy_t {
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

    // 平移牵引力
    struct {
        struct fxy_t {
            UnitFloat<N> ref, measure;
        } chassis, gimbal;
    } fx, fy;

    // 旋转牵引力，逆时针为正
    struct {
        UnitFloat<N> ref, measure;
    } fz;

    // 旋转力矩，逆时针为正
    struct {
        UnitFloat<N> ref, measure;
    } mr;

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
    void SetPowerLimit(const UnitFloat<>& power_limit);

    // 设置缓冲能量
    void SetBufferEnergy(const UnitFloat<>& buffer_energy);

    // 需要在循环中调用
    virtual void OnLoop();

protected:
    // 设置的值
    bool is_enable = false; // 底盘使能标志
    mode_e mode = DETACH_MODE;
    Angle<deg> gimbal_yaw;               // 前进正方向（云台相对于底盘的yaw）
    UnitFloat<W> power_limit = 120 * W;  // 功率限制
    UnitFloat<J> buffer_energy = 60 * J; // 缓冲能量

    // PID控制器
    PID vx_pid, vy_pid, vz_pid;
    PID follow_pid; // 底盘跟随PID

    void calcFollow();

    // 车体速度 -> 轮子速度
    virtual void forward() = 0;

    // 轮子速度 -> 车体速度
    virtual void backward() = 0;
};