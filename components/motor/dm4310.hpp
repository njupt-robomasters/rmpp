#pragma once

#include <cstdint>
#include "pid.hpp"
#include "unit.hpp"

class DM4310 {
public:
    struct {
        float torque = 0;
        Angle angle{};
        Speed speed{};
    } ref{}, measure{};

    DM4310(uint8_t id, PID::param_t &pid_param);

    void ParseCAN(const uint8_t data[8]);

    void ResetReady();

    [[nodiscard]] bool IsReady() const;

    void SetEnable(bool is_enable);

    void SetAngle(const Angle &angle);

    void SetAngle(const Angle &angle, const Speed &speed);

    void AddAngle(const Angle &angle);

    void Update();

    void SendCANEnable();

    void SendCANCmd();

private:
    constexpr static float P_MAX = 3.1415926f;
    constexpr static float V_MAX = 30.0f;
    constexpr static float KP_MAX = 500.0f;
    constexpr static float KD_MAX = 5.0f;
    constexpr static float T_MAX = 10.0f;

    // 电机CAN ID
    uint8_t motor_id;

    bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机失能标志
    uint32_t send_enable_cnt = 0;

    // CAN报文接收频率【单位：Hz】
    float can_recv_freq = 0;

    // 从CAN获得的电机数据
    struct motor_data_t {
        uint8_t id; // 电机ID
        char err; // 故障，8->超压，9->欠压，A->过电流，B->MOS过温，C->电机线圈过温，D->通讯丢失，E->过载
        uint16_t pos; // 位置【单位：rad】
        uint16_t vel; // 速度【单位：rad/s】
        uint16_t torque; // 扭矩【单位：N*m】
        uint8_t t_mos; // MOS温度【单位：℃】
        uint8_t t_rotor; // 电机线圈温度【单位：℃】
    } motor_data{};

    // 作用：计算CAN报文接收频率
    uint32_t dwt_cnt = 0;
    float dt = 0;

    PID pid;

    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
};
