#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"

class Motor {
  public:
    // 零量程与减速比在构造时取型号默认值；零点由外部维护
    struct Config {
        Can can;
        uint32_t tx_id = 0, rx_id = 0;
        float timeout = 0.1f;

        float max_current = 0, max_torque = 0, reduction = 0;

        bool is_invert = false;
        float offset = 0; // 电机侧零点，rad；单圈与多圈角度共用
    };

    Config config;

    bool is_connect = false;

    // 安装方向与减速比修正后的输出轴反馈与目标
    float current = 0;
    float current_ref = 0;
    float torque = 0;
    float torque_ref = 0;
    float speed = 0;
    float angle = 0;       // 单圈输出轴角度，[-π, π)
    float angle_multi = 0; // 连续输出轴角度，rad

  protected:
    bool is_enable = false;

    // 电机侧原始反馈，由协议子类解析
    float raw_current = 0, raw_torque = 0, raw_speed = 0, raw_angle = 0;

  private:
    // 多圈计数与反馈超时
    float raw_angle_multi = 0;
    float raw_angle_last = 0;
    bool is_angle_initialized = false;
    Timer callback_timer;

  public:
    virtual void set_enable(bool is_enable);

    // 设置原生输出：默认电流 A，DM 系列为力矩 N·m；PID 输出单位和限幅由 app 配置
    virtual void set_output(float output);

    virtual void set_current(float current);

    virtual void set_torque(float torque);

  protected:
    Motor(const Config& config, float max_current, float max_torque, float reduction);

    // 子类解析 raw 后调用；无角度的状态帧传 false，角度累计仅由有效角度帧推进
    void can_callback(bool is_angle_frame = true);

  public:
    // 每个控制周期调用，检查反馈是否超时
    void update();
};
