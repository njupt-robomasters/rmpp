#pragma once

#include "pid.hpp"
#include "unit.hpp"

class MF9025 {
private:
    // CAN通信参数
    const uint8_t can_port;
    const uint8_t motor_id; // 控制报文和反馈报文均为0x140+motor_id

    bool is_invert = false; // 电机反转标志
    volatile bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机使能标志

    PID pid;
    BSP::Dwt dwt; // 维护dt
    uint32_t can_cmd_cnt = 0; // 用于定期发送使能命令

    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    void sendCANEnable();

    void sendCANDisable();

    void sendCANReadState();

    void sendCANCmd();

public:
    // 力矩
    struct {
        Unit<A> ref, measure;
        int16_t raw = 0; // 电流原始值【-2048~2048对应-16.5A~16.5A】
    } current;

    // 角度
    struct {
        Angle<deg> ref, measure;
        uint16_t raw = 0; // 角度原始值【0~65535】
    } angle;

    // 转速
    struct {
        Unit<deg_s> ref, measure;
        int16_t raw = 0; // 转速原始值【单位：deg/s】
    } speed;

    Unit<C> temperate_motor; // 电机温度
    Unit<Hz> can_feedback_freq; // CAN反馈报文频率

    MF9025(uint8_t can_port, uint8_t motor_id);

    void SetInvert(bool is_invert);

    void SetPIDParam(PID::param_t* pid_param);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetAngle(Angle<deg> angle, Unit<deg_s> speed = 0);

    void Update();
};
