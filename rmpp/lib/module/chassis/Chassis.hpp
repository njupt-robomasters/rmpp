#pragma once

#include "algorithm/PID.hpp"
#include "math/Angle.hpp"
#include "math/Conversions.hpp"
#include "ui/UI.hpp"

using math::Angle;
using math::rpm;

class Chassis {
public:
    // SI: radius m, velocity m/s, angular velocity rad/s, force N, power W, energy J.
    // 底盘配置
    struct config_t {
        float chassis_radius{};                           // 底盘半径
        float wheel_radius{};                             // 轮子半径
        float reserve_wr = 60 * rpm;                      // 功率限制下，优先衰减旋转速度到保留值
        const PID::config_t* vxyz_pid_config = nullptr;   // 底盘运动PID参数
        const PID::config_t* follow_pid_config = nullptr; // 底盘跟随PID参数
    } config;

    bool is_enable = false; // 底盘使能标志

    // 底盘模式
    enum mode_e {
        DETACH_MODE, // 底盘分离模式
        FOLLOW_MODE, // 底盘跟随模式
    } mode = DETACH_MODE;

    // 移动速度
    // vx-前正后负，vy-左正右负
    struct {
        float ref{}, measure{};
    } vx, vy;

    // 旋转线速度，逆时针为正
    struct {
        float ref{}, measure{};
    } vz;

    // 旋转角速度，逆时针为正
    struct {
        float ref{}, measure{}, input{};
    } wr;

    // 移动牵引力
    // fx-前正后负，fy-左正右负
    struct {
        float ref{}, measure{};
    } fx, fy;

    // 旋转牵引力，逆时针为正
    struct {
        float ref{}, measure{};
    } fz;

    // 用于功率控制
    struct {
        float limit = 120;                                     // 功率限制
        float buffer_energy = 60;                              // 缓冲能量
        float gain{}, gain_x{}, gain_y{}, gain_xy{}, gain_z{}; // 力矩衰减系数
        float estimate{}, estimate_xy{}, estimate_z{};         // 功率估计
    } power;

    Chassis(const config_t& config);

    // 设置底盘使能/失能
    virtual void SetEnable(bool is_enable);

    // 设置底盘分离/跟随模式
    void SetMode(mode_e mode);

    // 设置底盘速度
    void SetSpeed(float vx, float vy, float wr);

    // 设置底盘前进正方向（云台yaw角度）
    void SetGimbalYaw(Angle gimbal_yaw);

    // 设置底盘功率限制，65535表示不限功率
    void SetPowerLimit(float power_limit);

    // 设置底盘缓冲能量
    void SetBufferEnergy(float buffer_energy);

    // 在自定义UI上显示电机连接状态
    virtual void UpdateUI(UI& ui) = 0;

    // 需要在循环中调用
    virtual void OnLoop();

protected:
    Angle gimbal_yaw; // 前进正方向（云台相对于底盘的yaw）

    // 计算PID
    void calcPID();

    // 功率控制，必须放在【逆解之前】
    void powerControl();

    // 速度和力学正解
    virtual void forward() = 0;

    // 速度和力学逆解
    virtual void backward() = 0;

private:
    PID vx_pid, vy_pid, vz_pid; // 底盘运动PID控制器
    PID follow_pid;             // 底盘跟随PID
};
