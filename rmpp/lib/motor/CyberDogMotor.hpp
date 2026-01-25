#pragma once

#include "Motor.hpp"

class CyberDogMotor : public Motor {
public:
    static constexpr float reduction = 1.0f;
    static constexpr UnitFloat<> Kt = 0.9f * Nm_A;
    static constexpr UnitFloat<> R = 0.17f * Ohm;

    UnitFloat<V> vbus; // 母线电压

    CyberDogMotor(const config_t& config);

    // 需要在循环中调用
    void OnLoop() override;

private:
    constexpr static float P_MAX = 12.5f;   // 位置最大值 rad
    constexpr static float V_MAX = 65.0f;   // 速度最大值 rad/s
    constexpr static float KP_MAX = 500.0f; // Kp最大值 Nm/rad
    constexpr static float KD_MAX = 5.0f;   // Kd最大值 Mm*s/rad
    constexpr static float I_MAX = 40.0f;   // 最大电流 A
    constexpr static float VB_MAX = 40.0f;  // 母体电压最大值 V

    // 用于定期发送使能命令
    uint32_t send_cnt = 0;

    // CAN接收函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    static int float_to_uint(float x, float x_min, float x_max, int bits);

    // 发送电机使能
    void sendEnable() const;

    // 发送电机失能
    void sendDisable() const;

    // 发送零位设置指令
    void sendZero() const;

    // 发送MIT控制指令
    void sendMIT() const;
};