#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"

class Motor {
  public:
    // 仅用于构造，零量程与减速比取型号默认值
    struct Config {
        Can can;
        uint32_t tx_id = 0, rx_id = 0;
        float timeout = 0.1f;

        float max_current = 0, max_torque = 0, reduction = 0;

        bool is_invert = false;
        float offset = 0;
    };

    bool is_connect = false;

    // 安装方向与减速比修正后的输出轴反馈与目标
    float current = 0;
    float current_ref = 0;
    float torque = 0;
    float torque_ref = 0;
    float speed = 0;
    float position = 0;

  private:
    float position_multi = 0;

  public:
    float offset;

    // 成对满量程，电流与力矩按二者之比换算；限流交给 PID max_out
    const float max_current, max_torque;

  protected:
    const Can can;
    const uint32_t tx_id, rx_id;
    const bool is_invert;
    const float reduction;

  private:
    const float timeout;

  protected:
    bool is_enable = false;

    // 电机侧原始反馈，由协议子类解析
    float raw_current = 0, raw_torque = 0, raw_speed = 0, raw_position = 0;

  private:
    // 多圈计数与反馈超时
    float raw_position_multi = 0;
    float raw_position_last = 0;
    Timer callback_timer;

  public:
    virtual void set_enable(bool is_enable);

    virtual void set_current(float current);

    virtual void set_torque(float torque);

    void set_offset(float offset);

    // 以当前位置为零点：按多圈位置换算并修改 offset
    void set_zero();

    // 每个控制周期调用，检查反馈是否超时
    void update();

  protected:
    Motor(const Config& config, float max_current, float max_torque, float reduction);

    // 子类校验并解析 raw 后调用，负责修正反馈和刷新在线状态；基类不注册回调
    void can_callback();
};
