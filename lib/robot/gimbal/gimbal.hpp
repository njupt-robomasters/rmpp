#pragma once

#include "bsp/kv_store.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"

// 云台公共部分：模式、目标转动、参考系换算与在线管理；电机组合与正逆解由子类实现
// 姿态由调用方每周期经 update() 传入云台系 IMU 的 yaw、pitch
class Gimbal {
  public:
    enum PidOutput {
        CURRENT_OUTPUT,
        TORQUE_OUTPUT,
    };

    struct Config {
        const Pid::Config* yaw_speed_pid_config;
        const Pid::Config* yaw_position_pid_config;
        const Pid::Config* pitch_speed_pid_config;
        const Pid::Config* pitch_position_pid_config;
        PidOutput yaw_output = CURRENT_OUTPUT;
        PidOutput pitch_output = CURRENT_OUTPUT;
        float pitch_min = 0;
        float pitch_max = 0;
        bool is_pitch_limit = false;
    };

    // 云台模式
    enum Mode {
        ECD_MODE, // 编码器模式
        IMU_MODE, // 陀螺仪模式
    };

    // 云台使能/失能
    void set_enable(bool is_enable);

    // 设置云台模式：编码器模式 / IMU模式
    void set_mode(Mode mode);

    // 设置速度
    void set_speed(float yaw_speed, float pitch_speed);

    // 设置角度
    void set_angle(Angle yaw, Angle pitch);

    // 设置底盘旋转速度，用于yaw轴前馈
    void set_chassis_wr(float chassis_wr);

    // 以当前 yaw 电机角度为零点
    virtual void set_yaw_zero() = 0;

    virtual void load_yaw_offset(KvStore& kv_store) = 0;
    virtual void save_yaw_offset(KvStore& kv_store) = 0;

    // 每个控制周期调用；imu_yaw、imu_pitch 为云台 IMU 姿态，rad
    void update(Angle imu_yaw, Angle imu_pitch);

  protected:
    explicit Gimbal(const Config& config);

    virtual void set_motors_enable(bool is_enable) = 0;

    // 离线检测
    virtual void update_motors() = 0;

    // 轴可控判断：失能或离线时目标跟随实测并清空 PID，恢复后第一个周期从实测重新开始
    // 须在 forward() 更新实测后调用，返回 false 时调用方输出零
    bool check_yaw(bool is_connect);
    bool check_pitch(bool is_connect);

    // 按配置的输出类型设置电流或力矩，返回电机限幅后的实际输出，供 PID 跟踪
    static float set_output(Motor& motor, PidOutput type, float output);

    // 角度正解：电机角度 -> 云台编码器角度
    virtual void forward() = 0;

    // 角度逆解：云台目标 -> 电机输出；dt 为控制周期，s
    virtual void backward(float dt) = 0;

  public:
    // 云台角度
    struct {
        struct {
            Angle ref, measure;
        } ecd, imu;

        Angle imu_minus_ecd;
    } yaw, pitch;

  protected:
    const Config config_;

    // 指令
    Mode mode_ = IMU_MODE;
    float yaw_speed_ = 0, pitch_speed_ = 0; // 云台转动速度，rad/s
    float chassis_wr_ = 0;                  // yaw速度前馈（小陀螺模式需要）

    Pid yaw_speed_pid_, yaw_position_pid_, pitch_speed_pid_, pitch_position_pid_;

  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s；避免首次调用和停顿后的大步长

    bool is_enable_ = false;
    bool is_yaw_ready_ = false, is_pitch_ready_ = false;
    Timer update_timer_;

    // 处理云台转动，按目标速度积分角度
    void handle_motion(float dt);
};
