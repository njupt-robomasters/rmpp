#pragma once

#include "math/angle.hpp"
#include "pid/pid.hpp"
#include "power/power_limiter.hpp"

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
        float chassis_radius = 0;                       // 底盘中心到轮的距离
        float wheel_radius = 0;                         // 轮子半径
        const Pid::Config* speed_pid_config = nullptr;  // 速度控制为轮速 PID（rad/s → A），力控制为底盘速度 PID（m/s → N）
        const Pid::Config* steer_pid_config = nullptr;  // 舵向 PID，误差 rad，输出电流 A
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

    std::array<Steer, 4> steer;

    // 使能/失能轮与舵电机
    void set_enable(bool is_enable);

    // 设置分离/跟随模式
    void set_mode(Mode mode);

    // 设置云台系平移速度与旋转角速度
    void set_speed(float vx, float vy, float wr);

    // 设置功率受限时优先衰减旋转的角速度阈值（力控制），默认 60 rpm
    void set_reserve_wr(float reserve_wr);

    // 设置前进正方向（云台相对底盘的 yaw）
    void set_gimbal_yaw(Angle gimbal_yaw);

    // 每个控制周期调用；失能时也更新测量与功率
    void update();

    // 运动学，底盘系 (x, y, z)；按实测舵向分解，速度取平均、力求和
    Vector3f forward_speed(const Vector4f& v) const; // 轮线速度 → 底盘速度
    Vector3f forward_force(const Vector4f& f) const; // 轮牵引力 → 底盘牵引力

  protected:
    const Config config_;
    const std::array<Motor*, 4> wheel_; // 轮电机
    PowerLimiter& power_;               // 功率限制，由 app 持有并设置裁判上限与缓冲能量
    bool is_enable_ = false;
    float reserve_wr_ = 60 * rpm; // 角速度超过此值时功率受限优先衰减旋转（力控制）
    Angle gimbal_yaw_;

    ChassisSteer(PowerLimiter& power, const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config);

    // 轮 i 能否驱动：舵电机离线时舵向不可控，对应轮停转
    bool can_drive(size_t i) const;

    // 速度逆解：底盘系速度目标 → 各舵目标并闭环，返回投影到实测舵向的轮线速度
    Vector4f backward_speed(const Vector3f& v);

    // 力学逆解：底盘系牵引力平分到各轮并投影到实测舵向
    Vector4f backward_force(const Vector3f& f) const;

    // 控制：底盘目标 → 电机输出；每周期调用，失能时清空 PID
    virtual void control() = 0;

  private:
    Mode mode_ = DETACH_MODE;
    Pid follow_pid_;
};

// 四舵轮底盘，速度控制：轮速 PID 输出电流，功率整体衰减
class ChassisSteerSpeed : public ChassisSteer {
  public:
    ChassisSteerSpeed(PowerLimiter& power, const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config);

  private:
    std::array<Pid, 4> wheel_pid_; // 轮转速误差 rad/s → 电流 A

    void control() override;
};

// 四舵轮底盘，力控制：底盘速度 PID 输出牵引力，力学逆解到实测舵向
class ChassisSteerTorque : public ChassisSteer {
  public:
    // 牵引力，云台系，fx 前正后负，fy 左正右负，fz 逆时针为正；ref 为功率与限幅后的实际输出
    struct {
        float ref = 0, measure = 0;
    } fx, fy, fz;

    ChassisSteerTorque(PowerLimiter& power, const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config);

  private:
    Pid vx_pid_, vy_pid_, vz_pid_; // 速度误差 m/s → 牵引力 N

    void control() override;
};
