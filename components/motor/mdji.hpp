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
    const Unit<A> current_max; // 最大电流
    const uint16_t current_cmd_max; // CAN通信最大电流对应的值

    float reduction; // 电机减速比

    // 电机反转标志
    bool is_invert = false;

    // 维护dt
    BSP::Dwt dwt;
    float dt = 0;

    // CAN反馈报文频率【单位：Hz】
    float can_feedback_freq = 0;

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

protected:
    PID pid;

public:
    bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机使能标志

    struct {
        Unit<A> ref, measure, measure_no_lfp;
        int16_t raw = 0; // 电流原始值
    } current{};

    struct {
        Angle<deg> ref, measure; // 输出轴角度
        uint16_t raw = 0; // 电机侧角度原始值【单位：0~8191 -> 0~360°】
    } angle{};

    struct {
        Unit<rpm> ref, measure, measure_no_lfp; // 输出轴转速
        int16_t raw = 0; // 电机侧转速原始值【单位：rpm】
    } speed{};

    Unit<C> temperate; // 电机温度

    MDJI(uint8_t can_port, uint32_t feedback_can_id,
         Unit<A> current_max, uint16_t current_cmd_max, float reduction);

    void SetInvert(bool is_invert);

    void SetReduction(float reduction);

    void SetPIDParam(PID::param_t& pid_param);

    void SetEnable(bool is_enable);

    void SetAngle(Angle<deg> angle);

    void SetSpeed(Unit<m_s> speed);

    int16_t GetCurrentCMD() const;
};
