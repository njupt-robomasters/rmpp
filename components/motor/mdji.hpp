#pragma once

#include "bsp.hpp"
#include "unit.hpp"
#include "pid.hpp"

class MDJI {
private:
    // 滤波参数
    static constexpr float CURRENT_LPF_FREQ = 10.0f; // 电流低通滤波参数【单位：Hz】
    static constexpr float SPEED_LPF_FREQ = 20.0f; // 转速低通滤波参数【单位：Hz】

    // CAN通信参数
    const uint8_t can_port; // CAN接口
    const uint32_t feedback_can_id; // CAN反馈报文ID
    const UnitFloat<A> current_max; // 最大电流
    const uint16_t current_cmd_max; // CAN通信最大电流对应的值

    bool is_invert = false; // 电机反转标志
    float reduction = 1.0f; // 电机减速比
    float current_ratio = 1.0f; // 用于功率控制

    BSP::Dwt dwt; // 维护dt

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

protected:
    volatile bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机使能标志
    PID pid;

public:
    // 电流
    struct {
        UnitFloat<A> ref, measure, measure_no_lfp;
        int16_t raw = 0; // 电流原始值
    } current;

    // 角度
    struct {
        Angle<deg> ref, measure; // 输出轴角度
        uint16_t raw = 0; // 电机侧角度原始值【单位：0~8191 -> 0~360°】
    } angle;

    // 转速
    struct {
        UnitFloat<rpm> ref, measure, measure_no_lfp; // 输出轴转速
        int16_t raw = 0; // 电机侧转速原始值【单位：rpm】
    } speed;

    // 其他
    UnitFloat<C> temperate_motor; // 电机温度
    UnitFloat<Hz> can_feedback_freq; // CAN反馈报文频率

    MDJI(uint8_t can_port, uint32_t feedback_can_id,
         UnitFloat<A> current_max, uint16_t current_cmd_max, float reduction);

    void SetReduction(float reduction);

    void SetInvert(bool is_invert);

    void SetCurrentRatio(float current_ratio);

    void SetPIDParam(PID::param_t* pid_param);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetAngle(Angle<deg> angle);

    void SetSpeed(UnitFloat<rpm> speed);

    int16_t GetCurrentCMD() const;
};
