#pragma once

#include "motor.hpp"

class DM4310 : public Motor {
  public:
    static constexpr float REDUCTION = 1.0f;
    static constexpr float KT = 1.2f;
    static constexpr float R = 0.65f / 2;
    static constexpr float MAX_TORQUE = 10.0f;
    static constexpr float MAX_CURRENT = MAX_TORQUE / KT;

    // 错误代码
    enum ErrCode {
        NONE = 0x00,
        OVP = 0x08,       // 超压
        UVP = 0x09,       // 欠压
        OCP = 0x0A,       // 过电流
        MOS_OTP = 0x0B,   // MOS过温
        MOTOR_OTP = 0x0C, // 电机过温
        COMM_LOST = 0x0D, // 通讯丢失
        OVERLOAD = 0x0E   // 过载
    } err_code = NONE;

    DM4310(const Config& config);

    void send_can_cmd();

  private:
    constexpr static float P_MAX = 3.14159f; // 为了多圈计数，一定要在上位机改成这个值（其他参数都是默认值）
    constexpr static float V_MAX = 30.0f;
    constexpr static float KP_MAX = 500.0f;
    constexpr static float KD_MAX = 5.0f;
    constexpr static float T_MAX = 10.0f;

    uint32_t send_cnt_ = 0; // 用于定期发送使能命令

    // CAN接收函数
    void callback(uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    // 发送电机使能
    void send_enable() const;

    // 发送电机失能
    void send_disable() const;

    // 发送电流
    void send_torque(float torque) const;
};