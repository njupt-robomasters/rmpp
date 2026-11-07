#pragma once

#include "chassis_power.hpp"
#include "math/angle.hpp"
#include "math/eigen_types.hpp"
#include "math/unit.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"
#include <array>

// 单个舵向：舵电机、舵向 PID 与目标
class Steer {
  public:
    Motor& motor;

    // 舵向，0 朝前，逆时针为正
    Angle ref, measure;

    Steer(Motor& motor, const Pid::Config* pid_config);

    // 离线检测并读取舵向
    void update();

    // 按轮速度矢量设置目标舵向并闭环，返回投影到当前舵向的轮速，m/s；离线时返回零
    float control(float vx, float vy);

    // 清空 PID，下次控制从当前舵向开始
    void clear();

  private:
    static constexpr float MIN_V = 1 * cm; // 最小转舵速度，m/s

    Pid pid_; // 舵向误差 rad → 电流 A
    bool has_ref_ = false;
};

// 四舵轮底盘公共部分：模式与跟随、云台系换算、轮与舵在线管理、舵向正逆解、功率
// 轮与舵顺序：左前、左后、右后、右前；正方形布置
// 控制方式由 ChassisSteerSpeed、ChassisSteerTorque 实现；接口与 ChassisMatrix 一致
// 国际单位制：半径 m，速度 m/s，角速度 rad/s，力 N
class ChassisSteer {
  public:
    struct Config {
        float chassis_radius{};                         // 底盘中心到轮的距离
        float wheel_radius{};                           // 轮子半径
        float reserve_wr = 60 * rpm;                    // 角速度超过此值时功率受限优先衰减旋转（力控制）
        const Pid::Config* follow_pid_config = nullptr; // 跟随 PID，误差 rad，输出 rad/s；为空时跟随模式不旋转
        const Pid::Config* steer_pid_config = nullptr;  // 舵向 PID，误差 rad，输出电流 A
        ChassisPower::Config power;                     // 功率估计与超电，不含舵电机
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

    // 功率：由调用方设置裁判上限与缓冲能量
    ChassisPower power;

    std::array<Steer, 4> steer;

    const Config& config() const;

    // 使能/失能，切换时回到分离模式，防止车突然转动
    void set_enable(bool is_enable);

    // 设置分离/跟随模式
    void set_mode(Mode mode);

    // 设置云台系平移速度与旋转角速度
    void set_speed(float vx, float vy, float wr);

    // 设置前进正方向（云台相对底盘的 yaw）
    void set_gimbal_yaw(Angle gimbal_yaw);

    // 每个控制周期调用；失能时也更新测量与功率
    void update();

    // 运动学，底盘系 (x, y, z)；按实测舵向分解，速度取平均、力求和
    Vector3 forward(const Vector4& v) const;       // 轮线速度 → 底盘速度
    Vector3 forward_force(const Vector4& f) const; // 轮牵引力 → 底盘牵引力

  protected:
    const Config config_;
    const std::array<Motor*, 4> wheel_; // 轮电机

    ChassisSteer(const Config& config, const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer);

    // 轮 i 能否驱动：舵电机离线时舵向不可控，对应轮停转
    bool can_drive(size_t i) const;

    // 速度逆解：底盘系速度目标 → 各舵目标并闭环，返回投影到实测舵向的轮线速度
    Vector4 backward(const Vector3& v);

    // 力学逆解：底盘系牵引力平分到各轮并投影到实测舵向
    Vector4 backward_force(const Vector3& f) const;

    // 云台系与底盘系互转，只旋转 x、y
    Vector3 to_chassis(const Vector3& gimbal) const;
    Vector3 to_gimbal(const Vector3& chassis) const;

    // 控制：底盘目标 → 电机输出；每周期调用，失能时清空 PID
    virtual void control() = 0;

  private:
    Pid follow_pid_;
    Angle gimbal_yaw_;
};

// 四舵轮底盘，速度控制：轮速 PID 输出电流，功率整体衰减
class ChassisSteerSpeed : public ChassisSteer {
  public:
    ChassisSteerSpeed(const Config& config, const Pid::Config* wheel_pid_config,
                      const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer);

  private:
    std::array<Pid, 4> wheel_pid_; // 轮转速误差 rad/s → 电流 A

    void control() override;
};

// 四舵轮底盘，力控制：底盘速度 PID 输出牵引力，力学逆解到实测舵向
class ChassisSteerTorque : public ChassisSteer {
  public:
    // 牵引力，云台系，fx 前正后负，fy 左正右负，fz 逆时针为正；ref 为功率与限幅后的实际输出
    struct {
        float ref{}, measure{};
    } fx, fy, fz;

    ChassisSteerTorque(const Config& config, const Pid::Config* vxyz_pid_config,
                       const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer);

  private:
    Pid vx_pid_, vy_pid_, vz_pid_; // 速度误差 m/s → 牵引力 N

    void control() override;
};
