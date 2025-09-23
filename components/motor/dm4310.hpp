#pragma once

#include "pid.hpp"
#include "unit.hpp"

class DM4310 {
private:
    constexpr static float P_MAX = 3.14159f;
    constexpr static float V_MAX = 30.0f;
    constexpr static float KP_MAX = 500.0f;
    constexpr static float KD_MAX = 5.0f;
    constexpr static float T_MAX = 10.0f;

    // CAN通信参数
    const uint8_t can_port;
    const uint32_t master_id, slave_id;

    bool is_invert = false; // 电机反转标志
    volatile bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机使能标志

    PID pid;
    BSP::Dwt dwt; // 维护dt
    uint32_t can_cmd_cnt = 0; // 用于定期发送使能命令

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    void sendCANEnable();

    void sendCANDisable();

    void sendCANCmd();

public:
    // 力矩
    struct {
        UnitFloat<Nm> ref, measure;
        uint16_t raw = 0; // 力矩原始值【12bit数据表示-T_MAX~T_MAX，单位N*m】
    } torque;

    // 角度
    struct {
        Angle<deg> ref, measure;
        uint16_t raw = 0; // 角度原始值【16bit数据表示-P_MAX~P_MAX，单位rad】
    } angle;

    // 转速
    struct {
        UnitFloat<deg_s> ref, measure;
        uint16_t raw = 0; // 转速原始值【12bit数据表示-V_MAX~V_MAX，单位rad/s】
    } speed;

    uint8_t err_code = 0; // 故障代码，8->超压，9->欠压，A->过电流，B->MOS过温，C->电机过温，D->通讯丢失，E->过载
    UnitFloat<C> temperate_mos; // MOS温度
    UnitFloat<C> temperate_motor; // 电机温度
    UnitFloat<Hz> can_feedback_freq; // CAN反馈报文频率

    DM4310(uint8_t can_port, uint32_t master_id, uint32_t slave_id);

    void SetInvert(bool is_invert);

    void SetPIDParam(PID::param_t* pid_param);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetAngle(Angle<deg> angle, UnitFloat<deg_s> speed = 0);

    void Update();
};
