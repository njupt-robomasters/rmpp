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

        // 为零的字段取型号默认值；max_current 与 max_torque 为成对的满量程，二者之比即转矩常数，限流交给 PID 的 max_out
        struct Drive {
            float max_current = 0;
            float max_torque = 0;
            float reduction = 0;
        } drive;

        // offset 可由 set_offset() 运行时修改
        struct Install {
            bool is_invert = false;
            float offset = 0;
        } install;
    };

    struct Raw {
        float current = 0, torque = 0, speed = 0, position = 0;
    };

    virtual void set_enable(bool is_enable);
    virtual void set_current(float current);
    virtual void set_torque(float torque);
    void set_offset(float offset);

    // 以当前位置为零点：按多圈位置换算并修改 offset
    void set_zero();

    // 每个控制周期调用，检查反馈是否超时
    void update();

  protected:
    // model 为型号默认驱动参数，补齐 config.drive 中为零的字段
    Motor(const Config& config, const Config::Drive& model);

    // 子类校验并解析 raw 后调用，负责修正反馈和刷新在线状态；基类不注册回调
    void can_callback();

  public:
    bool is_connect = false;

    // 安装方向与减速比修正后的输出轴反馈与目标
    float current = 0;
    float current_ref = 0;
    float torque = 0;
    float torque_ref = 0;
    float speed = 0;
    float position = 0;
    float position_multi = 0;

    Config config;

    Raw raw; // 电机侧原始反馈，子类解析写入

  protected:
    bool is_enable_ = false;

  private:
    float raw_position_multi_ = 0;
    float raw_position_last_ = 0;
    Timer callback_timer_;

    static Config with_model(Config config, const Config::Drive& model);
};
