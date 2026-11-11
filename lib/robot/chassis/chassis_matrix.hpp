#pragma once

#include "math/angle.hpp"
#include "pid/pid.hpp"
#include "power/motor_power.hpp"
#include "power/super_cap.hpp"
#include <array>

// 矩阵四轮底盘公共部分：模式与跟随、云台系换算、轮电机在线管理、矩阵正逆解、功率
// 速度逆解 J、正解 J⁺；力正解 Jᵀ、逆解 (J⁺)ᵀ
// 控制方式由 ChassisMatrixSpeed、ChassisMatrixTorque 实现
// 国际单位制：半径 m，速度 m/s，角速度 rad/s，力 N
class ChassisMatrix {
  public:
    // 轮向固定的四轮运动学矩阵 J：每行一个轮，轮线速度 = 行 · (vx, vy, vz)，vz 为底盘半径处旋转线速度
    // 轮序：左前、左后、右后、右前；底盘系 x 前、y 左、逆时针为正
    // 全向轮，45° 布置
    static constexpr float OMNI[4][3] = {
        {-std::sqrt(0.5f), +std::sqrt(0.5f), 1},
        {-std::sqrt(0.5f), -std::sqrt(0.5f), 1},
        {+std::sqrt(0.5f), -std::sqrt(0.5f), 1},
        {+std::sqrt(0.5f), +std::sqrt(0.5f), 1},
    };

    // 麦克纳姆轮
    static constexpr float MECANUM[4][3] = {
        {-1, +1, std::sqrt(2.0f)},
        {-1, -1, std::sqrt(2.0f)},
        {+1, -1, std::sqrt(2.0f)},
        {+1, +1, std::sqrt(2.0f)},
    };

    struct Config {
        const float (*kinematics)[3] = nullptr;         // 运动学矩阵 4×3，OMNI 或 MECANUM
        float chassis_radius = 0;                       // 底盘中心到轮的距离
        float wheel_radius = 0;                         // 轮子半径
        float wheel_resistance = 0;                     // 轮电机线间电阻，Ω，用于功率估计
        const Pid::Config* speed_pid_config = nullptr;  // 速度控制为轮速 PID（rad/s → A），力控制为底盘速度 PID（m/s → N）
        const Pid::Config* follow_pid_config = nullptr; // 跟随 PID，误差 rad，输出 rad/s；为空时跟随模式不旋转
    };

    // 底盘模式
    enum Mode {
        DETACH_MODE, // 分离模式
        FOLLOW_MODE, // 跟随模式
    };

    // 移动速度，云台系，vx 前正后负，vy 左正右负
    struct {
        float ref = 0, measure = 0;
    } vx, vy;

    // 旋转线速度，逆时针为正
    struct {
        float ref = 0, measure = 0;
    } vz;

    // 旋转角速度，逆时针为正；input 为用户输入，ref 叠加跟随后得到
    struct {
        float ref = 0, measure = 0, input = 0;
    } wr;

    // 功率：轮电机模型估计与本周期平移、旋转电流衰减系数；底盘功率目标由超电给出
    struct {
        float estimate = 0;
        MotorPower::Gain gain;
    } power;

    // 使能/失能轮电机
    void set_enable(bool is_enable);

    // 设置分离/跟随模式
    void set_mode(Mode mode);

    // 设置云台系平移速度与旋转角速度
    void set_speed(float vx, float vy, float wr);

    // 设置功率受限时优先衰减旋转的角速度阈值，默认 60 rpm
    void set_reserve_wr(float reserve_wr);

    // 设置前进正方向（云台相对底盘的 yaw）
    void set_gimbal_yaw(Angle gimbal_yaw);

    // 每个控制周期调用；失能时也更新测量与功率
    void update();

    // 运动学，底盘系 (x, y, z)，均为纯函数
    Vector3f forward_speed(const Vector4f& v) const;  // 轮线速度 → 底盘速度
    Vector4f backward_speed(const Vector3f& v) const; // 底盘速度 → 轮线速度
    Vector3f forward_force(const Vector4f& f) const;  // 轮牵引力 → 底盘牵引力
    Vector4f backward_force(const Vector3f& f) const; // 底盘牵引力 → 轮牵引力

  protected:
    const Config config_;
    Motor* const wheel_[4]; // 轮电机：左前、左后、右后、右前
    SuperCap& super_cap_;   // 由底盘更新，提供底盘功率目标
    bool is_enable_ = false;
    float reserve_wr_ = 60 * rpm; // 角速度超过此值时功率受限优先衰减旋转
    Angle gimbal_yaw_;

    ChassisMatrix(SuperCap& super_cap, const std::array<Motor*, 4>& wheel, const Config& config);

    // 控制：底盘目标 → 电机输出；每周期调用，失能时清空 PID
    virtual void control() = 0;

  private:
    Mode mode_ = DETACH_MODE;
    Eigen::Matrix<float, 4, 3> j_;
    Eigen::Matrix<float, 3, 4> j_pinv_;
    Pid follow_pid_;
};

// 矩阵四轮底盘，速度控制：速度逆解为各轮线速度，轮速 PID 输出电流
// 电流按力正解拆出旋转分量做功率控制，假设四轮同型号
class ChassisMatrixSpeed : public ChassisMatrix {
  public:
    ChassisMatrixSpeed(const std::array<Motor*, 4>& wheel, SuperCap& super_cap, const Config& config);

  private:
    Pid wheel_pid_[4]; // 轮转速误差 rad/s → 电流 A

    void control() override;
};

// 矩阵四轮底盘，力控制：底盘速度 PID 输出牵引力，力学逆解为各轮电流
class ChassisMatrixTorque : public ChassisMatrix {
  public:
    // 牵引力，云台系，fx 前正后负，fy 左正右负，fz 逆时针为正；ref 为功率与限幅后的实际输出
    struct {
        float ref = 0, measure = 0;
    } fx, fy, fz;

    ChassisMatrixTorque(SuperCap& super_cap, const std::array<Motor*, 4>& wheel, const Config& config);

  private:
    Pid vx_pid_, vy_pid_, vz_pid_; // 速度误差 m/s → 牵引力 N

    void control() override;
};
