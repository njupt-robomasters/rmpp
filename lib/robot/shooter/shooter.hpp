#pragma once

#include "math/unit.hpp"
#include "motor/motor.hpp"
#include "pid/pid.hpp"
#include "referee/ui.hpp"
#include <stdint.h>

// 发射机构公共部分；摩擦轮与拨弹电机各自处理掉线，互不耦合，便于调试
class Shooter {
  public:
    enum PidOutput { CURRENT_OUTPUT,
                     TORQUE_OUTPUT };

    // 发射机构配置
    struct Config {
        const Pid::Config* rub_pid_config;
        const Pid::Config* shoot_pid_config;
        PidOutput rub_output = CURRENT_OUTPUT;
        PidOutput shoot_output = CURRENT_OUTPUT;
        float bullet_per_angle = 1.0f / rev;  // 每弧度发射弹数，默认每转一发
        float block_time = 500 * ms;          // 卡弹检测时间
        float reverse_time = 250 * ms;        // 卡弹倒转时间
        float rub_radius = 3.0f * cm;         // 摩擦轮半径
        float auto_release_speed = 120 * rpm; // 停止且低速时释放摩擦轮，rad/s；零禁用
        uint16_t heat_protect = 0;            // 剩余热量保护阈值
    } config;

    bool is_enable = false;       // 使能标志
    bool is_rub = false;          // 摩擦轮状态
    bool is_shoot = false;        // 拨弹电机状态
    bool is_heat_protect = false; // 热量保护状态

    // 弹速 (m/s)
    struct {
        float ref{}, measure{};
    } bullet_speed;

    // 弹频 (shots/s)
    struct {
        float ref{}, measure{};
    } bullet_freq;

    // 拨弹电机电流
    float shoot_current{};

    Shooter(const Config& config);

    // 设置发射结构使能/失能
    void set_enable(bool is_enable);

    // 设置弹速参考值，m/s，保持到下次设置
    void set_bullet_speed(float bullet_speed);

    // 设置弹频参考值，发/秒，保持到下次设置
    void set_bullet_freq(float bullet_freq);

    // 设置摩擦轮
    void set_rub(bool is_rub);

    // 设置拨弹电机
    void set_shoot(bool is_shoot);

    // 传入裁判剩余热量，允许过热时的负值
    void set_heat_remain(int32_t heat_remain);

    // 在自定义UI上显示电机连接状态
    virtual void update_ui(RobotUi& ui) = 0;

    // 需要在循环中调用
    void update();

  protected:
    Pid shoot_pid_;
    bool is_block_ = false; // 堵转标志
    Timer timer_block_;     // 堵转计时器

    virtual void set_motors_enable(bool is_enable) = 0;

    // 离线检测
    virtual void update_motors() = 0;

    // 摩擦轮速度闭环；离线或关闭时清空 PID 并输出零，关闭且低速时释放
    void control_rub(Motor& motor, Pid& pid, float speed_ref, float speed_measure);

    // 按配置的输出类型设置电流或力矩
    static void set_output(Motor& motor, PidOutput type, float output);

    // 正解：电机速度 -> 弹速、弹频
    virtual void forward() = 0;

    // 逆解：弹速、弹频 -> 电机速度
    virtual void backward() = 0;

  private:
    int32_t heat_remain_ = INT32_MAX; // 独立示例未接入裁判时不限制
};
