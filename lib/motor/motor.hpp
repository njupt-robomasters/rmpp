#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"

class Motor {
  public:
    struct Config {
        struct Bus {
            Can can;
            uint32_t tx_id = 0, rx_id = 0;
            float timeout = 0.1f;
        } const bus;

        struct Drive {
            float max_current = 0;
            float max_torque = 0;
            float kt = 1;
            float reduction = 1;
        } drive;

        struct Install {
            bool is_invert = false;
            float offset = 0;
        } install;
    } config;

    bool is_connect = false;

    float current = 0;
    float current_ref = 0;
    float torque = 0;
    float torque_ref = 0;
    float speed = 0;
    float position = 0;
    float position_multi = 0;

    struct Raw {
        float current = 0, torque = 0, speed = 0, position = 0;
    } raw;

    virtual void set_enable(bool is_enable);
    virtual void set_current(float current);
    virtual void set_torque(float torque);
    void set_offset(float offset);

    // 每个控制周期调用，检查反馈是否超时
    void update();

  protected:
    bool is_enable_ = false;

    explicit Motor(const Config& config);

    // 子类校验并解析 raw 后调用，负责修正反馈和刷新在线状态；基类不注册回调
    void can_callback();

  private:
    Timer rx_timer_;
    float raw_position_multi_ = 0;
    float raw_position_last_ = 0;
};
