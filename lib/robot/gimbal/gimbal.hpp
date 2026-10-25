#pragma once

#include "bsp/storage.hpp"
#include "imu/imu.hpp"
#include "math/angle.hpp"
#include "pid/pid.hpp"
#include "referee/ui.hpp"

class Gimbal {
  public:
    enum PidOutput { CURRENT_OUTPUT, TORQUE_OUTPUT };

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
    Imu& imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    bool is_enable = false; // 云台使能标志

    // 云台模式
    enum Mode {
        ECD_MODE, // 编码器模式
        IMU_MODE, // 陀螺仪模式
    } mode = IMU_MODE;

    // 云台角度
    struct {
        struct {
            Angle ref, measure;
        } ecd, imu;

        Angle imu_minus_ecd;
    } yaw, pitch;

    float yaw_speed{}, pitch_speed{}; // 云台速度

    Gimbal(Imu& imu, const Config& config);

    // 云台使能/失能
    virtual void set_enable(bool is_enable);

    // 设置云台模式：编码器模式 / IMU模式
    void set_mode(Mode mode);

    // 设置速度
    void set_speed(float yaw_speed, float pitch_speed);

    // 设置角度
    void set_angle(Angle yaw, Angle pitch);

    // 设置底盘旋转速度，用于yaw轴前馈
    void set_chassis_wr(float chassis_wr);

    virtual void load_yaw_offset(Storage& storage) = 0;

    virtual void save_yaw_offset(Storage& storage) = 0;

    virtual void set_yaw_zero() = 0;

    // 在自定义UI上显示电机连接状态
    virtual void update_ui(RobotUi& ui) = 0;

    // 需要在循环中调用
    virtual void update();

  protected:
    float chassis_wr_{}; // yaw速度前馈（小陀螺模式需要）
    const Config config_;
    Pid yaw_speed_pid_, yaw_position_pid_, pitch_speed_pid_, pitch_position_pid_;

    // 角度正解：电机角度 -> 云台姿态
    virtual void forward() = 0;

    // 角度正解：云台姿态 -> 电机角度
    virtual void backward() = 0;

  private:
    Timer timer_motion_; // 用于云台转动

    // 处理云台转动，通过dt不断计算角度增量
    void handle_motion();
};
