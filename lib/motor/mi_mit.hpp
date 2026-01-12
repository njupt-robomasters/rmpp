#pragma once

#include "motor.hpp"

class MiMIT : public Motor {
public:
    // 从CAN读取的额外信息
    UnitFloat<C> temperate_mos;   // MOS温度
    UnitFloat<C> temperate_motor; // 电机温度

    MiMIT(uint8_t can_port, uint32_t master_id, uint32_t slave_id);

    // 需要在循环中调用
    void OnLoop();
    // 发送MIT控制指令
    void sendMIT() const;
    // 设置控制参数（用于位置速度模式）
    void SetPositionControl(float kp, float kd);

    // 发送零位设置指令
    void sendZero() const;

private:
    // MiMIT电机参数限制
    constexpr static float P_MIN = -12.5f;  // 位置最小值 rad
    constexpr static float P_MAX = 12.5f;   // 位置最大值 rad
    constexpr static float V_MIN = -20.5f;  // 速度最小值 rad/s
    constexpr static float V_MAX = 20.5f;   // 速度最大值 rad/s
    constexpr static float KP_MIN = 0.0f;   // Kp最小值
    constexpr static float KP_MAX = 100.0f; // Kp最大值
    constexpr static float KD_MIN = 0.0f;   // Kd最小值
    constexpr static float KD_MAX = 5.0f;   // Kd最大值
    constexpr static float I_MAX = 40.0f;   // 最大电流 A
    constexpr static float KT = 0.1f;       // 扭矩常数 Nm/A
    constexpr static float GR = 9.0f;       // 减速比

    // CAN通信参数
    const uint8_t can_port;
    const uint32_t master_id, slave_id;

    // 用于定期发送使能命令
    uint32_t send_cnt = 0;

    // MIT模式控制参数
    float kp_ = 0.0f;
    float kd_ = 0.0f;

    // CAN接收函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    static int float_to_uint(float x, float x_min, float x_max, int bits);

    // 发送电机使能
    void sendEnable() const;

    // 发送电机失能
    void sendDisable() const;
};