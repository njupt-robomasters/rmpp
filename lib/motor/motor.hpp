#pragma once

#include "bsp/can.hpp"
#include "pid.hpp"

class Motor {
  public:
    // 控制模式
    enum ControlMode {
        OPEN_LOOP_MODE,                  // 开环模式（电流/力矩模式）
        SPEED_MODE,                      // 速度模式
        SINGLE_TURN_POSITION_MODE,       // 单圈位置模式
        SINGLE_TURN_POSITION_SPEED_MODE, // 单位置-速度串级模式
        MULTI_TURN_POSITION_MODE,        // 多圈位置模式
        MULTI_TURN_POSITION_SPEED_MODE,  // 多圈位置-速度串级模式
    };

    // PID输出类型
    enum PidOutputType {
        CURRENT_OUTPUT, // 电流输出
        TORQUE_OUTPUT,  // 力矩输出
    };

    struct Config {
        struct Bus {
            Can can;                       // CAN 外设
            uint32_t tx_id = 0, rx_id = 0; // 发送、接收 CAN ID
            float timeout = 0.1f;          // 反馈超时，s
        } const bus;

        // 零值采用型号默认值
        struct Drive {
            float kt = 0;          // 输出轴转矩常数，N*m/A
            float max_current = 0; // 电流协议量程，A
            float max_torque = 0;  // 转矩协议量程，N*m
            float reduction = 0;   // 电机轴到输出轴的机械减速比
        } drive;

        struct Install {
            float offset = 0;       // 电机轴零点偏移，rad，可运行时校准
            bool is_invert = false; // 输出轴方向反转
        } install;

        struct Control {
            ControlMode mode = OPEN_LOOP_MODE;                // 开环、速度或位置闭环
            PidOutputType pid_output_type = CURRENT_OUTPUT;   // PID 输出单位
            const Pid::Config* speed_pid_config = nullptr;    // 速度环参数
            const Pid::Config* position_pid_config = nullptr; // 位置环参数
            bool is_limit = false;                            // 是否限制位置目标
            float position_min = 0, position_max = 0;         // 位置目标范围，rad
        } control;

        Config with_defaults(const Drive& drive) const;
    };

    bool is_connect = false;
    bool is_enable = false;

    struct {
        float ref = 0, measure = 0;
    } current, torque;

    // ref 为速度环目标，位置级联时等于位置环输出加 ff；ff 为位置控制的速度前馈
    struct {
        float ref = 0, ff = 0, measure = 0;
    } speed;

    struct {
        float ref = 0, measure = 0, measure_multi = 0;
    } position;

    // 修正前反馈，由子类解析写入；速度为输出轴 rad/s，位置为电机轴 rad
    struct Raw {
        float current = 0, torque = 0, speed = 0, position = 0;
    } raw;

    virtual void set_enable(bool is_enable);

    void set_current(float current);
    void set_torque(float torque);
    void set_speed(float speed);
    float set_position(float position, float speed_ff = 0); // 返回实际设置的位置

    void set_offset(float offset);

    const Config& config() const {
        return config_;
    }

    // 每个控制周期调用：先刷新在线状态，再计算闭环
    void update();

  protected:
    Config config_;

    explicit Motor(const Config& config);

    // 子类校验并解析 raw 后调用，负责修正反馈和刷新在线状态；基类不注册回调
    void can_callback();

  private:
    Timer rx_timer_;
    Pid speed_pid_, position_pid_;

    // 跨周期状态
    float raw_position_multi_ = 0;
    float raw_position_last_ = 0;
};
