#pragma once

#include "motor.hpp"

class DM4310 : public Motor {
public:
    static constexpr UnitFloat Kt = 1.2f * Nm_A;
    static constexpr float R = 0.65f / 2; // 相电阻（两相电阻/2）

    // CAN通信参数
    const uint8_t can_port;
    const uint32_t master_id, slave_id;

    enum err_code_e {
        NONE      = 0x00,
        OVP       = 0x08, // 超压
        UVP       = 0x09, // 欠压
        OCP       = 0x0A, // 过电流
        MOS_OTP   = 0x0B, // MOS过温
        MOTOR_OTP = 0x0C, // 电机过温
        COMM_LOST = 0x0D, // 通讯丢失
        OVERLOAD  = 0x0E  // 过载
    };

    // 从CAN读取的电机额外信息
    err_code_e err_code = NONE;   // 错误代码
    UnitFloat<C> temperate_mos;   // MOS温度
    UnitFloat<C> temperate_motor; // 电机温度

    DM4310(uint8_t can_port, uint32_t master_id, uint32_t slave_id);

    // 需要在循环中调用
    void OnLoop();

private:
    constexpr static float P_MAX = 3.14159f; // 为了多圈计数，一定要在上位机改成这个值（其他参数都是默认值）
    constexpr static float V_MAX = 30.0f;
    constexpr static float KP_MAX = 500.0f;
    constexpr static float KD_MAX = 5.0f;
    constexpr static float T_MAX = 10.0f;

    // 用于定期发送使能命令
    uint32_t send_cnt = 0;

    // CAN接收函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    // 发送电机使能
    void sendEnable() const;

    // 发送电机失能
    void sendDisable() const;

    // 发送MIT控制指令
    void sendMIT() const;
};