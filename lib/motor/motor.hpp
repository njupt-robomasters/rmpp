#pragma once

#include "bsp/can.hpp"
#include "math/angle.hpp"
#include "pid.hpp"

class Motor {
  public:
    enum ControlMode {
        OPEN_LOOP_MODE,
        SPEED_MODE,
        SINGLE_TURN_MODE,
        SINGLE_TURN_SPEED_MODE,
        MULTI_TURN_MODE,
        MULTI_TURN_SPEED_MODE,
    };

    enum PidOutputType {
        CURRENT_OUTPUT,
        TORQUE_OUTPUT,
    };

    // 单位为 s、rad、rad/s、A、N*m、N*m/A
    struct Config {
        struct Protocol {
            Can can;                              // CAN 通道
            uint32_t master_id = 0, slave_id = 0; // 反馈、命令 ID
            float timeout = 100 * ms;             // 反馈超时，s
        } const protocol;

        struct Physical {
            float kt = 0;          // 输出转矩常数，N*m/A；零采用型号默认值
            float max_current = 0; // 电流协议量程，A；零采用型号默认值
            float max_torque = 0;  // 转矩协议量程，N*m；零采用型号默认值
        } physical;

        struct Install {
            float offset = 0;       // 电机轴安装偏移，rad，可运行时校准
            bool is_invert = false; // 输出轴方向反转
            float reduction = 0;    // 减速比；零采用型号默认值
        } install;

        struct Control {
            ControlMode control_mode = OPEN_LOOP_MODE;        // 开环、速度或位置闭环
            PidOutputType pid_output_type = CURRENT_OUTPUT;   // PID 输出单位
            const Pid::Config* speed_pid_config = nullptr;    // 速度环参数
            const Pid::Config* position_pid_config = nullptr; // 位置环参数
            bool is_limit = false;                            // 是否限制位置目标
            float position_min = 0, position_max = 0;         // 位置目标范围，rad
        } control;
    } config;

    bool is_connected = false;
    bool is_enabled = false;

    struct {
        float ref = 0, measure = 0;
    } current, torque;

    struct {
        float ref = 0, outer = 0, measure = 0;
    } speed;

    struct {
        float ref = 0, measure = 0, measure_multi = 0;
    } position;

    struct {
        float mos = 0, motor = 0;
    } temperature;

    // 修正前反馈；速度已换算为输出轴 rad/s，位置为电机轴 rad
    struct Raw {
        float current = 0, torque = 0, speed = 0, position = 0;
    } raw;

    virtual ~Motor() = default;
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    Motor(Motor&&) = delete;
    Motor& operator=(Motor&&) = delete;

    virtual void set_enable(bool enable);
    void set_current(float current);
    void set_torque(float torque);
    void set_speed(float speed);
    float set_position(float position, float speed_ff = 0);
    bool set_offset(float offset);
    bool set_position_zero();
    virtual void update_control();

  protected:
    explicit Motor(const Config& config);
    static Config apply_defaults(Config config, const Config& defaults);
    void subscribe();
    virtual void receive(const CanFrame& frame) = 0;
    bool is_feedback_frame(const CanFrame& frame) const;
    void update_connection();
    void mark_received();
    void accept_feedback(const Raw& feedback);
    void record_position(float sample);
    float output_current(float maximum) const;
    float output_torque(float maximum) const;

  private:
    Timer timer_connect_;
    Pid speed_pid_, position_pid_;
    float last_position_sample_ = 0;
    float accumulated_position_ = 0;
    bool has_position_sample_ = false;
    bool position_pending_ = true;
    bool is_multi_turn() const;
    float selected_position() const;
    void refresh_position();
    void clear_control();
    float output(float value, float maximum) const;
};
