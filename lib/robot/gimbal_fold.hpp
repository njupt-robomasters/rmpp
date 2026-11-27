#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 折叠哨兵云台：大小 yaw + 折叠关节 pitch1 + 俯仰轴 pitch2
// yaw 目标为头部世界航向，pitch 目标为头部世界俯仰；编码器提供关节角、限位和底盘前进方向
// - 小 yaw 为主执行器：头部角度外环 → 头部世界角速度内环（陀螺）→ 电流，吃掉大 yaw 尚未转到的部分
// - 大 yaw 为辅助轴：自身世界角外环 → 自身世界角速度内环 → 力矩；识别窗口内保持，超限补足，普通模式把小 yaw 拉回中位
// - pitch2：世界目标换算到编码器角度外环 → 电机编码器速度内环，叠加重力补偿力矩
// - pitch1 折叠关节当前固定失能，保留测量，控制输出为零；目标接口预留给后续启用
// yaw 需要大小 yaw 同时在线，恢复首周期输出零；pitch2 独立在线即可闭环，恢复首周期从实测开始
// 国际单位制：角度 rad，角速度 rad/s，力矩 Nm，电流 A
class GimbalFold {
  public:
    // 重力补偿：力矩 = torque·cos(angle + phase)，只在该电机可控时叠加；torque 为零不补偿
    struct Gravity {
        float torque = 0; // 质心位于水平时的重力力矩，Nm，符号与电机正方向一致
        float phase = 0;  // 质心连线与该轴零位的夹角，rad
    };

    struct Config {
        // 大 yaw：世界角度 → 世界角速度 → 力矩
        const Pid::Config* yaw1_angle_pid_config;
        const Pid::Config* yaw1_speed_pid_config;

        // 小 yaw：头部世界角度 → 头部世界角速度（IMU）→ 电流；限位为相对大 yaw 的角度，rad
        const Pid::Config* yaw2_angle_pid_config;
        const Pid::Config* yaw2_speed_pid_config;
        float yaw2_min = 0;
        float yaw2_max = 0;

        // 俯仰：世界目标换算到编码器角度 → 电机速度 → 力矩；限位为 pitch2 编码器角，rad
        const Pid::Config* pitch_angle_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        float pitch_min = 0;
        float pitch_max = 0;

        // 折叠关节：当前失能，接口保留
        const Pid::Config* pitch1_position_pid_config;
        const Pid::Config* pitch1_speed_pid_config;
        float pitch1_min = 0;
        float pitch1_max = 0;

        Gravity pitch1_gravity; // angle 取 pitch1 编码器角度
        Gravity pitch2_gravity; // angle 取 IMU pitch，云台头装在 pitch1 上，绝对俯仰只能从 IMU 得到

        // 角度外环最大角加速度，rad/s²；无穷大时禁用加速度规划
        float yaw_max_accel = INFINITY;
        float pitch_max_accel = INFINITY; // 保留配置接口，pitch2 编码器闭环不使用加速度规划
    };

    // 云台总 yaw：世界系目标与实测；编码器总角供底盘前进方向
    struct {
        Angle ref, measure;                     // 头部世界 yaw，rad
        float speed_ref = 0, speed_measure = 0; // 世界角速度，rad/s；ref 含角度外环修正
        Angle ecd_measure;                      // yaw1 + yaw2 编码器角，rad
        Angle imu_minus_ecd;                    // 世界角与编码器总角之差，rad
    } yaw;

    // 云台 pitch：世界系目标与实测；编码器角供限位，扫描端点取 ecd_ref
    struct {
        Angle ref, measure;                     // 头部世界 pitch，rad
        float speed_ref = 0, speed_measure = 0; // pitch2 电机角速度，rad/s
        Angle ecd_ref, ecd_measure;             // 编码器角，rad；ecd_ref 为世界目标换算并限位后的值
        Angle imu_minus_ecd;                    // 世界角与编码器角之差，rad
    } pitch;

    // 大 yaw：ref、world_measure 为自身世界角，measure 为编码器角
    struct {
        Angle ref;                              // 自身世界 yaw 目标，rad；小 yaw 中位为 0 时等于头部目标
        Angle measure;                          // 编码器角，rad
        Angle world_measure;                    // 世界 yaw = 头部世界角 − 小 yaw 编码器角，rad
        float speed_ref = 0, speed_measure = 0; // 世界角速度，rad/s
    } yaw1;

    // 小 yaw 编码器角度
    struct {
        Angle measure;
    } yaw2;

    // 折叠关节编码器角度
    struct {
        Angle ref, measure;
    } pitch1;

    // 云台电机
    struct Motors {
        Motor &yaw1, &yaw2;     // 大 yaw、小 yaw
        Motor &pitch1, &pitch2; // 折叠关节、俯仰
    };

  private:
    const Config config;
    const Motors motor;

    // 指令
    bool is_enable = false;
    bool is_yaw_window = false;           // 识别窗口：大轴保持世界朝向，小轴优先追踪，超出限位时大轴补足
    float yaw_speed = 0, pitch_speed = 0; // 云台目标转速，rad/s，世界系

    Pid yaw1_angle_pid, yaw1_speed_pid;
    Pid yaw2_angle_pid, yaw2_speed_pid;
    Pid pitch_angle_pid, pitch_speed_pid;
    Pid pitch1_position_pid, pitch1_speed_pid;

    bool is_yaw_ready = false, is_pitch_ready = false;
    bool is_yaw_window_ready = false;
    Angle yaw1_world_ref; // 窗口模式下大轴保持的世界朝向，rad
    Timer update_timer;

  public:
    GimbalFold(const Motors& motor, const Config& config);

    void set_enable(bool is_enable);

    void set_speed(float yaw_speed, float pitch_speed);

    // 设置头部世界 yaw/pitch 目标并清零速度指令，rad
    void set_angle(Angle yaw, Angle pitch);

    // 自瞄识别到目标时启用小 yaw 优先窗口；窗口范围共用小 yaw 编码器软限位
    void set_yaw_window(bool is_yaw_window);

    // 世界 yaw 目标相对转动 delta，rad；pitch 不变
    void turn_yaw(float delta);

    // 保存折叠关节目标角；当前失能策略在 update 中将其恢复为实测角
    void set_pitch1_angle(Angle angle);

    // 每个控制周期调用；q 为云台到世界的单位四元数，gyro 为安装修正后的云台系角速度，rad/s
    void update(const Quaternionf& q, const Vector3f& gyro);

  private:
    // 按最大角加速度折算外环等效角度误差，大误差下走平方根制动曲线；无穷大时原样返回
    static float plan_angle_error(float err, const Pid::Config* pid_config, float max_accel);

    // 俯仰与折叠关节重力补偿力矩，Nm
    static float gravity_output(const Gravity& gravity, float angle);
};
