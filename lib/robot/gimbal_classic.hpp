#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 单 yaw、单 pitch 云台，机构为底盘→yaw→pitch
// IMU 固连 pitch 随动部分，安装修正后 X 沿枪口、Y 沿 pitch 正轴；pitch 编码器零位对应两级坐标系对齐
// 内环：IMU 角速度经 pitch 关节角换算到电机轴，yaw、pitch 目标与反馈均为绝对角速度
// 外环（可选）：世界 yaw/pitch 角度闭环，输出的世界角速率修正换算到电机轴后叠加到速度指令
class GimbalClassic {
  private:

  public:
    struct Motors {
        Motor &yaw, &pitch;
    } motor;

    struct Config {
        // PID 参数
        const Pid::Config* yaw_speed_pid_config;
        const Pid::Config* yaw_angle_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        const Pid::Config* pitch_angle_pid_config;

        // pitch 编码器限位，rad；外环按 IMU 与编码器偏差换算后钳制世界目标，越限时禁止继续向外输出
        float pitch_min = 0;
        float pitch_max = 0;

        // 角度外环最大角加速度，rad/s²；无穷大时禁用加速度规划
        float yaw_max_accel = INFINITY;
        float pitch_max_accel = INFINITY;
    };

  private:
    const Config config;

    // 指令
    bool is_enable = false;
    float yaw_speed = 0, pitch_speed = 0; // 电机轴速度，rad/s；外环开启时同时积分到世界角度目标

  public:
    // 角度为枪口世界 yaw/pitch；速度为 yaw、pitch 轴绝对角速度，rad/s
    struct Axis {
        Angle angle_ref, angle_measure;
        float speed_ref = 0, speed_measure = 0; // ref 与 measure 为闭环目标与反馈；目标含外环修正，不可控时为 0
    } yaw, pitch;

  private:
    Pid yaw_speed_pid, yaw_angle_pid, pitch_speed_pid, pitch_angle_pid;
    bool has_update = false; // 首次 update 前角度目标未同步实测

    Timer update_timer;

  public:
    GimbalClassic(const Motors& motor, const Config& config);

    // 云台使能/失能
    void set_enable(bool is_enable);

    // 设置沿电机轴的绝对角速度指令，rad/s
    void set_speed(float yaw_speed, float pitch_speed);

    // 设置世界 yaw/pitch 目标并清零速度指令，rad；关闭角度环时目标跟随实测
    void set_angle(Angle yaw, Angle pitch);

    // q 为云台到世界的单位四元数；gyro 为安装修正后的云台系角速度，rad/s
    void update(const Quaternionf& q, const Vector3f& gyro);

  private:
    // 角度外环：积分角度目标，修正量换算到电机轴后叠加到 speed_ref；不可控时目标跟随实测并清空 PID
    void update_angle_loop(const Quaternionf& q, const Quaternionf& pitch_joint, bool can_yaw, bool can_pitch, float dt);

    // 按最大角加速度折算外环等效角度误差，大误差下走平方根制动曲线；无穷大时原样返回
    static float plan_angle_error(float err, const Pid::Config* pid_config, float max_accel);
};
