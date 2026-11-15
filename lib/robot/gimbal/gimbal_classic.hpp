#pragma once

#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 单 yaw、单 pitch 云台
// 姿态由调用方每周期经 update() 传入云台系 IMU 的 yaw、pitch
class GimbalClassic {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s

  public:
    // 云台模式
    enum Mode {
        ECD_MODE, // 编码器模式
        IMU_MODE, // 陀螺仪模式
    };

    // 云台角度
    struct {
        struct {
            Angle ref, measure;
        } ecd, imu;

        Angle imu_minus_ecd;
    } yaw, pitch;

    struct Config {
        const Pid::Config* yaw_speed_pid_config;
        const Pid::Config* yaw_position_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        const Pid::Config* pitch_position_pid_config;
        float pitch_min = 0;
        float pitch_max = 0;
    } const config;

  private:
    struct Motors {
        Motor &yaw, &pitch;
    } motor;

    Pid yaw_speed_pid, yaw_position_pid, pitch_speed_pid, pitch_position_pid;

    // 指令
    Mode mode = IMU_MODE;
    float yaw_speed = 0, pitch_speed = 0; // 云台转动速度，rad/s
    float chassis_wr = 0;                 // yaw 速度前馈，rad/s

    bool is_enable = false;
    bool is_yaw_ready = false, is_pitch_ready = false;
    Timer update_timer;

  public:
    GimbalClassic(const Motors& motor, const Config& config);

    // 云台使能/失能
    void set_enable(bool is_enable);

    // 设置云台模式：编码器模式 / IMU 模式
    void set_mode(Mode mode);

    // 设置速度，rad/s
    void set_speed(float yaw_speed, float pitch_speed);

    // 设置角度，rad
    void set_angle(Angle yaw, Angle pitch);

    // 设置底盘旋转速度，用于 yaw 轴前馈，rad/s
    void set_chassis_wr(float chassis_wr);

    // 每个控制周期调用；imu_yaw、imu_pitch 为云台 IMU 姿态，rad
    void update(Angle imu_yaw, Angle imu_pitch);
};
