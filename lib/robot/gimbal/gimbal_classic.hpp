#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 单 yaw、单 pitch 云台，机构为底盘→yaw→pitch
// IMU 固连 pitch 随动部分，安装修正后 X 沿枪口、Y 沿 pitch 正轴；pitch 编码器零位对应两级坐标系对齐
// 内环：IMU 角速度经 pitch 关节角换算到 yaw、pitch 电机轴后闭环
// 外环（可选）：世界 yaw/pitch 角度闭环，输出的世界角速率修正换算到电机轴后叠加到速度指令
class GimbalClassic {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s

  public:
    struct Config {
        // PID 参数
        const Pid::Config* yaw_speed_pid_config;
        const Pid::Config* yaw_angle_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        const Pid::Config* pitch_angle_pid_config;

        // pitch 编码器保护范围，rad；越限时禁止继续向外输出
        float pitch_min = 0;
        float pitch_max = 0;

        bool use_angle_loop = true; // false 时只有速度内环
    } const config;

    // 角度为枪口世界 yaw/pitch；速度为电机轴绝对角速度，rad/s；ref 与 measure 即闭环的目标与反馈
    struct Axis {
        Angle angle_ref, angle_measure;
        float speed_ref = 0, speed_measure = 0; // 目标含外环修正，yaw 另含底盘前馈；不可控时目标为 0
    } yaw, pitch;

    struct Motors {
        Motor &yaw, &pitch;
    } motor;

  private:
    // 指令
    bool is_enable = false;
    float yaw_speed = 0, pitch_speed = 0; // 电机轴速度，rad/s；外环开启时同时积分到世界角度目标
    float chassis_wr = 0;                 // 底盘绕 yaw 轴转速，rad/s

    Pid yaw_speed_pid, yaw_angle_pid, pitch_speed_pid, pitch_angle_pid;
    bool has_update = false; // 首次 update 前角度目标未同步实测

    Timer update_timer;

  public:
    GimbalClassic(const Motors& motor, const Config& config);

    // 云台使能/失能
    void set_enable(bool is_enable);

    // 设置电机轴速度，rad/s
    void set_speed(float yaw_speed, float pitch_speed);

    // 设置世界 yaw/pitch 目标并清零速度指令，rad；关闭角度环时目标跟随实测
    void set_angle(Angle yaw, Angle pitch);

    // 设置底盘绕 yaw 轴的转速，rad/s；从 yaw 速度目标中扣除，抵消底盘带动
    void set_chassis_wr(float chassis_wr);

  private:
    // 角度外环：积分角度目标，修正量换算到电机轴后叠加到 speed_ref；不可控时目标跟随实测并清空 PID
    void update_angle_loop(const Quaternionf& q, const Quaternionf& pitch_joint, bool can_yaw, bool can_pitch, float dt);

  public:
    // q 为云台到世界的单位四元数；gyro 为安装修正后的云台系角速度，rad/s
    void update(const Quaternionf& q, const Vector3f& gyro);
};
