#pragma once

#include "motor/motor.hpp"
#include <cstdint>

// 灵足 EL05，私有扩展帧 1 Mbps、8 字节，tx_id 为电机 ID，rx_id 为主机 ID（均为 8 位）
// p、v、kp、kd 置零，发送力矩前馈；位置、速度闭环由机构在主控完成
class EL05 : public Motor {
  public:
    static constexpr float MAX_CURRENT = 11.0f;
    static constexpr float MAX_TORQUE = 6.0f;
    static constexpr float REDUCTION = 1;

  private:
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 50.0f;
    static constexpr float T_MAX = 6.0f;

    enum Command : uint8_t {
        TORQUE = 1,
        FEEDBACK = 2,
        ENABLE = 3,
        STOP = 4,
    };

  public:
    enum Mode : uint8_t {
        RESET_MODE,
        CALIBRATION_MODE,
        MOTOR_MODE,
    };

    Mode mode = RESET_MODE;
    uint8_t fault = 0;              // 扩展 ID bit21~16，六项故障标志
    uint32_t clear_fault_count = 0; // 自动清错发送次数

  private:
    bool enable_sent = false;
    bool has_fault = false;
    Timer enable_timer;
    Timer clear_fault_timer;

  public:
    explicit EL05(const Config& config);

    // 私有运控协议原生输出力矩，N·m
    void set_output(float output) override;

    // 每周期调用，停止帧也会返回状态；故障时每 500 ms 尝试清错
    void send();

  private:
    void can_callback(const CanFrame& frame);
    void send_torque() const;
    void send_command(Command command, bool clear_fault = false) const;

    static float decode(uint16_t value, float maximum);
    static uint16_t encode(float value, float maximum);
};
