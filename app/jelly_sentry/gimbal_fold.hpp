#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 折叠哨兵云台：大小 yaw + 折叠关节 pitch1 + 俯仰轴 pitch2
// - 大 yaw 持续跟随头部总目标，小 yaw 补偿大 yaw 尚未转到的部分，随大轴跟上回中
// - pitch1 折叠关节当前固定失能，保留测量，控制输出为零；目标接口预留给后续启用
// - pitch2 为传统俯仰轴，IMU 装在其上；pitch1 转动时 IMU 模式经 imu_minus_ecd 自动补偿
// - pitch2 叠加重力补偿力矩 torque·cos(angle + phase)，pitch1 补偿当前停用
// 各电机离线只影响自身：输出零并清空自身 PID，恢复后从实测重新闭环
// 国际单位制：角度 rad，角速度 rad/s，力矩 Nm，电流 A
class GimbalFold {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s

  public:
    enum Mode {
        ECD_MODE,
        IMU_MODE,
    };

    // 独立串级轴：位置环误差 rad → 速度 rad/s，速度环输出单位与电机原生输出一致
    struct Axis {
        const Pid::Config* speed_pid_config = nullptr;
        const Pid::Config* position_pid_config = nullptr;
        float min = 0; // 下限位，编码器角度，rad
        float max = 0; // 上限位，编码器角度，rad
    };

    // 重力补偿：力矩 = torque·cos(angle + phase)，只在该电机可控时叠加；torque 为零不补偿
    struct Gravity {
        float torque = 0; // 质心位于水平时的重力力矩，Nm，符号与电机正方向一致
        float phase = 0;  // 质心连线与该轴零位的夹角，rad
    };

    struct Config {
        const Pid::Config* yaw_speed_pid_config;
        const Pid::Config* yaw_position_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        const Pid::Config* pitch_position_pid_config;
        float pitch_min = 0;
        float pitch_max = 0;

        Axis yaw2;              // 小 yaw，限位为相对大 yaw 的角度
        Axis pitch1;            // 折叠关节
        Gravity pitch1_gravity; // angle 取 pitch1 编码器角度
        Gravity pitch2_gravity; // angle 取 IMU pitch，云台头装在 pitch1 上，绝对俯仰只能从 IMU 得到
    };

    // 云台总角度，分别记录编码器系与 IMU 系
    struct {
        struct {
            Angle ref, measure;
        } ecd, imu;

        Angle imu_minus_ecd;
    } yaw, pitch;

    // 云台电机
    struct Motors {
        Motor &yaw1, &yaw2;     // 大 yaw、小 yaw
        Motor &pitch1, &pitch2; // 折叠关节、俯仰
    };

    // 大小 yaw 分配与折叠关节角度，均为各自电机的编码器角度
    struct {
        Angle ref, measure;
    } yaw1, yaw2, pitch1;

  private:
    const Config config;
    const Motors motor;
    Pid yaw_speed_pid, yaw_position_pid, pitch_speed_pid, pitch_position_pid;
    Pid yaw2_speed_pid, yaw2_position_pid;
    Pid pitch1_speed_pid, pitch1_position_pid;
    Mode mode = IMU_MODE;
    float yaw_speed = 0, pitch_speed = 0; // 云台目标转速，rad/s
    float chassis_wr = 0;                 // 底盘旋转前馈，rad/s
    bool is_enable = false;
    bool is_yaw_ready = false, is_pitch_ready = false;
    bool is_yaw1_ready = false, is_yaw2_ready = false;
    Timer update_timer;

  public:
    GimbalFold(const Motors& motor, const Config& config);

    void set_enable(bool is_enable);
    void set_mode(Mode mode);
    void set_speed(float yaw_speed, float pitch_speed);
    void set_angle(Angle yaw, Angle pitch);
    void turn_yaw(float delta);
    void set_chassis_wr(float chassis_wr);

    // 保存折叠关节目标角；当前失能策略在 update 中将其恢复为实测角
    void set_pitch1_angle(Angle angle);

    // 每个控制周期调用；输入云台 IMU 姿态，rad
    void update(Angle imu_yaw, Angle imu_pitch);

  private:
    // 单电机可控判断：失能或离线时清空 PID 并返回 false；恢复后第一个周期仍返回 false
    bool check_motor(const Motor& motor, bool& is_ready, Pid& speed_pid, Pid& position_pid) const;

    // 俯仰与折叠关节重力补偿力矩，Nm
    static float gravity_output(const Gravity& gravity, float angle);
};
