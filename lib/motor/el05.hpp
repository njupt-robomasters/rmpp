#pragma once

#include "motor/motor.hpp"
#include <cstdint>

// 灵足 RobStride EL05 一体化关节电机，私有协议 CAN 2.0 扩展帧 1 Mbps
// 扩展 ID：bit28~24 通信类型，bit23~8 数据区 2（主机 ID 或力矩指令），bit7~0 目标电机 ID
// tx_id 为电机 CAN ID（出厂 0x7F），rx_id 为主机 ID（出厂 0xFD）
// 电机工作在运控模式且 p、v、kp、kd 置零，只接收力矩前馈；位置、速度闭环由机构在主控完成
class EL05 : public Motor {
  public:
    // 型号默认物理参数：峰值电流、峰值力矩
    static constexpr float MAX_CURRENT = 11.0f;
    static constexpr float MAX_TORQUE = 6.0f;
    static constexpr float KT = 0.94f;
    static constexpr float REDUCTION = 1;

    // 出厂 ID
    static constexpr uint8_t DEFAULT_MOTOR_ID = 0x7F;
    static constexpr uint8_t DEFAULT_HOST_ID = 0xFD;

    // 反馈帧中的电机模式
    enum Mode : uint8_t {
        RESET_MODE,       // 复位（未使能）
        CALIBRATION_MODE, // 标定
        MOTOR_MODE,       // 运行
    };

    Mode mode = RESET_MODE;
    uint8_t fault = 0;     // 故障位，非零时只发停止帧
    float temperature = 0; // 电机温度，℃

    explicit EL05(const Config& config);

    // 每个控制周期调用：失能或故障时发停止帧；使能时按 ENABLE_FREQUENCY 补发使能帧，其余周期发力矩
    void send();

  private:
    // 协议量程：角度 rad、速度 rad/s、力矩 Nm，均为对称区间
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 50.0f;
    static constexpr float T_MAX = 6.0f;

    static constexpr float ENABLE_FREQUENCY = 10; // 使能帧补发频率，Hz

    // 通信类型
    enum Type : uint8_t {
        REMOTE_CONTROL = 1, // 运控指令
        FEEDBACK = 2,       // 电机反馈
        ENABLE = 3,         // 使能
        STOP = 4,           // 停止
    };

    Timer enable_timer_;

    void can_callback(const CanFrame& frame);

    void send_torque() const;
    void send_enable() const;
    void send_stop() const;

    // 组合发送帧的扩展 ID
    uint32_t extend_id(Type type, uint16_t data) const;

    static float decode(uint16_t value, float maximum);
    static uint16_t encode(float value, float maximum);
};
