#pragma once

#include "gimbal.hpp"

// 折叠哨兵云台：大小 yaw + 折叠关节 pitch1 + 俯仰轴 pitch2
// - 大 yaw（yaw1）跟随云台总目标，小 yaw（yaw2）补偿大 yaw 尚未转到的部分并限位；两者各自串级 PID，输出类型可分别配置
// - pitch1 为折叠关节，旋转抬起整个云台头，只用编码器闭环，目标由 set_pitch1_angle() 给出，未设置时保持使能时的角度
// - pitch2 为传统俯仰轴，IMU 装在其上，走基类的 ECD / IMU 模式；pitch1 转动时 IMU 模式经 imu_minus_ecd 自动补偿
// - pitch1、pitch2 可叠加重力补偿力矩 torque·cos(angle + phase)
// 各电机离线只影响自身：输出零并清空自身 PID，恢复后第一个周期仍输出零，下个周期从实测重新闭环
// 国际单位制：角度 rad，角速度 rad/s，力矩 Nm，电流 A
class GimbalFold : public Gimbal {
  public:
    // 独立串级轴：位置环误差 rad → 速度 rad/s，速度环误差 rad/s → 电流 A 或力矩 Nm
    struct Axis {
        const Pid::Config* speed_pid_config = nullptr;
        const Pid::Config* position_pid_config = nullptr;
        PidOutput output = CURRENT_OUTPUT;
        float min = 0; // 下限位，编码器角度，rad
        float max = 0; // 上限位，编码器角度，rad
    };

    // 重力补偿：力矩 = torque·cos(angle + phase)，只在该电机可控时叠加；torque 为零不补偿
    struct Gravity {
        float torque = 0; // 质心位于水平时的重力力矩，Nm，符号与电机正方向一致
        float phase = 0;  // 质心连线与该轴零位的夹角，rad
    };

    struct Config {
        Gimbal::Config gimbal;  // yaw_* 为大 yaw，pitch_* 与限位为俯仰轴 pitch2
        Axis yaw2;              // 小 yaw，限位为相对大 yaw 的角度
        Axis pitch1;            // 折叠关节
        Gravity pitch1_gravity; // angle 取 pitch1 编码器角度
        Gravity pitch2_gravity; // angle 取 IMU pitch，云台头装在 pitch1 上，绝对俯仰只能从 IMU 得到
    };

    // 云台电机
    struct Motors {
        Motor &yaw1, &yaw2;     // 大 yaw、小 yaw
        Motor &pitch1, &pitch2; // 折叠关节、俯仰
    } motor;

    // 大小 yaw 分配与折叠关节角度，均为各自电机的编码器角度
    struct {
        Angle ref, measure;
    } yaw1, yaw2, pitch1;

    GimbalFold(Attitude& attitude, const Motors& motor, const Config& config);

    // 设置折叠关节目标角，转动速度受 pitch1 位置环 max_out 限制，超出限位时截断
    void set_pitch1_angle(Angle angle);

    void load_yaw_offset(KvStore& kv_store) override;

    void save_yaw_offset(KvStore& kv_store) override;

    void set_yaw_zero() override;

  private:
    const Config fold_config_;
    Pid yaw2_speed_pid_, yaw2_position_pid_;
    Pid pitch1_speed_pid_, pitch1_position_pid_;
    bool is_motors_enable_ = false;
    bool is_yaw1_ready_ = false, is_yaw2_ready_ = false, is_pitch1_ready_ = false;

    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度逆解：云台姿态 -> 电机输出
    void backward() override;

    // 单电机可控判断：失能或离线时清空 PID 并返回 false；恢复后第一个周期仍返回 false
    bool check_motor(const Motor& motor, bool& is_ready, Pid& speed_pid, Pid& position_pid) const;

    // 重力补偿换算到电机输出类型的单位
    static float gravity_output(const Motor& motor, PidOutput output, const Gravity& gravity, float angle);
};
