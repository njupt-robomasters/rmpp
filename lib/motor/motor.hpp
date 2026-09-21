#pragma once

#include "algorithm/pid.hpp"
#include "math/angle.hpp"
#include "math/conversions.hpp"

using math::Angle;
using math::ms;

class Motor {
  public:
    // 电机闭环模式
    enum ControlMode {
        OPEN_LOOP_MODE,   // 开环电流/力矩控制
        SPEED_MODE,       // 速度闭环
        ANGLE_MODE,       // 角度闭环
        ANGLE_SPEED_MODE, // 角度-角度闭环（串级PID）
    };

    // PID输出类型
    enum PidOutType {
        CURRENT_OUTPUT, // PID输出电流
        TORQUE_OUTPUT,  // PID输出扭矩
    };

    // SI values: s, rad, rad/s, A, N*m, N*m/A, ohm. Temperatures remain Celsius.
    // 电机配置
    struct Config {
        // CAN通信参数
        uint8_t can_port = -1;
        uint32_t master_id = 0, slave_id = 0;
        float timeout = 100.0f * ms; // 电机断联超时时间

        // 电机参数
        float reduction = 0.0f; // 减速比
        float kt = 0.0f;        // 力矩系数
        float r = 0.0f;         // 相电阻（两相间电阻/2）

        // 电机安装参数
        bool is_invert = false;     // 电机反转标志
        Angle offset;               // 电机角度偏移量
        bool is_limit = false;      // false-圆周模式，true-限位模式
        Angle limit_min, limit_max; // min-右限位（负值），max-左限位（正值），相对于offset的角度

        // PID相关
        ControlMode control_mode = OPEN_LOOP_MODE;     // 电机闭环模式
        PidOutType pid_out_type = CURRENT_OUTPUT;      // PID输出类型
        const Pid::Config* speed_pid_config = nullptr; // 速度PID参数
        const Pid::Config* angle_pid_config = nullptr; // 角度PID参数
    } config;

    bool is_connect = false; // 电机连接状态
    bool is_enable = false;  // 电机使能状态

    // 电流
    struct {
        float ref{}, measure{};
    } current;

    // 扭矩
    struct {
        float ref{}, measure{};
    } torque;

    // 转速
    struct {
        float ref{}, outer{}, measure{};
    } speed;

    // 减速比!=1时累积编码器角增量，存储值仍归一化到 [-pi, pi)。
    struct {
        Angle ref, measure;
    } angle;

    // 温度
    struct {
        float mos{}, motor{};
    } temperature;

    // 电机反转、偏移修正之前的值
    struct Raw {
        float current{};
        float torque{};
        float speed{};
        Angle angle;
    } raw;

    Motor(const Config& config);

    // 设置电机使能/失能
    void set_enable(bool is_enable);

    // 设置电流
    void set_current(float current);

    // 设置扭矩
    void set_torque(float torque);

    // 设置速度
    void set_speed(float speed);

    // 设置角度
    Angle set_angle(Angle angle, float speed_ff = 0.0f);

    // 需要在循环中调用
    virtual void on_loop();

  protected:
    rmpp::Timer timer_connect_; // 用于电机断联检测

    // CAN接收回调，子类重写后要调用
    void callback(const Raw& raw);

  private:
    Pid speed_pid_, angle_pid_;      // PID控制器
    Angle last_angle_;               // 用于多圈计数
    bool is_first_setangle_ = false; // 断联/使能后第一次设置角度标志，防止电机一下子飞起来
};
