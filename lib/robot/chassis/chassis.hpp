#pragma once

#include "math/angle.hpp"
#include "math/unit.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"
#include "referee/ui.hpp"
#include <array>
#include <cstddef>
#include <tuple>

// 四轮底盘公共部分：模式与跟随、云台系换算、轮电机在线管理、功率输入与估计
// 控制方式由 ChassisSpeed、ChassisTorque 实现，构型由其子类实现
// 国际单位制：半径 m，速度 m/s，角速度 rad/s，力 N，功率 W，能量 J
class Chassis {
  public:
    struct Config {
        float chassis_radius{};                         // 底盘中心到轮的距离
        float wheel_radius{};                           // 轮子半径
        float wheel_resistance{};                       // 轮电机线间电阻，Ω，用于功率估计与控制
        float reserve_wr = 60 * rpm;                    // 功率受限时优先衰减旋转，保留的角速度
        const Pid::Config* follow_pid_config = nullptr; // 跟随 PID，误差 rad，输出 rad/s；为空时跟随模式不旋转
    };

    // 底盘模式
    enum Mode {
        DETACH_MODE, // 分离模式
        FOLLOW_MODE, // 跟随模式
    };

    bool is_enable = false;
    Mode mode = DETACH_MODE;

    // 移动速度，云台系，vx 前正后负，vy 左正右负
    struct {
        float ref{}, measure{};
    } vx, vy;

    // 旋转线速度，逆时针为正
    struct {
        float ref{}, measure{};
    } vz;

    // 旋转角速度，逆时针为正；input 为用户输入，ref 叠加跟随后得到
    struct {
        float ref{}, measure{}, input{};
    } wr;

    // 功率
    struct {
        float limit = 120;        // 功率限制，65535 表示不限
        float buffer_energy = 60; // 缓冲能量
        float estimate{};         // 估计功率：轮电机机械功率与铜耗，不含舵电机
    } power;

    const Config& config() const;

    // 使能/失能，切换时回到分离模式，防止车突然转动
    void set_enable(bool is_enable);

    // 设置分离/跟随模式
    void set_mode(Mode mode);

    // 设置云台系平移速度与旋转角速度
    void set_speed(float vx, float vy, float wr);

    // 设置前进正方向（云台相对底盘的 yaw）
    void set_gimbal_yaw(Angle gimbal_yaw);

    // 设置功率限制，65535 表示不限功率
    void set_power_limit(float power_limit);

    // 设置缓冲能量
    void set_buffer_energy(float buffer_energy);

    // 在自定义 UI 上显示电机连接状态
    virtual void update_ui(RobotUi& ui);

    // 每个控制周期调用
    void update();

  protected:
    const Config config_;
    const std::array<Motor*, 4> wheel_; // 轮电机：左前、左后、右后、右前
    Angle gimbal_yaw_;

    Chassis(const Config& config, const std::array<Motor*, 4>& wheel);

    virtual void set_motors_enable(bool is_enable);

    // 离线检测
    virtual void update_motors();

    // 轮 i 能否驱动，不能驱动的轮由控制层清空 PID 并输出零
    virtual bool can_drive(size_t i) const;

    // 失能时每周期清空 PID
    virtual void clear_pid();

    // 轮 i 线速度与牵引力，离线时为零
    float wheel_speed(size_t i) const;
    float wheel_force(size_t i) const;

    // 单个轮电机铜耗，current 为 iq
    float copper_loss(float current) const;

    // 云台系速度目标换算到底盘系 (vx, vy, vz)
    std::tuple<float, float, float> speed_ref() const;

    // 底盘系速度测量换算到云台系
    void set_speed_measure(float vx, float vy, float vz);

    // 正解：电机反馈 → 底盘测量
    virtual void forward() = 0;

    // 控制：底盘目标 → 电机输出
    virtual void control() = 0;

  private:
    Pid follow_pid_;
};
