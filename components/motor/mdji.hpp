#pragma once

#include <cstdint>
#include "pid.hpp"
#include "unit.hpp"

class MDJI {
public:
    struct ref_data_t {
        float current = 0;
        Angle angle{};
        Speed speed{};
    } ref;

    struct measure_data_t {
        float current = 0;
        float current_raw = 0;
        Angle angle{};
        Speed speed{};
        Speed speed_raw{};
    } measure;

    MDJI(float current_max, uint16_t can_cmd_max, float reduction_ratio, PID::param_t &pid_param);

    void ParseCAN(const uint8_t data[8]);

    void ResetReady();

    [[nodiscard]] bool IsReady() const;

    void SetEnable(bool is_enable);

    void SetAngle(const Angle &angle, const Speed &speed);

    void SetAngle(const Angle &angle);

    void SetSpeed(const Speed &speed);

    [[nodiscard]] int16_t GetCurrentCMD() const;

protected:
    bool is_ready = false; // 电机就绪标志，收到CAN反馈报文后置为true
    bool is_enable = false; // 电机使能标志
    PID pid;

private:
    static constexpr float CURRENT_LPF_FREQ = 10.0f; // 电流转低通滤波参数【单位：Hz】
    static constexpr float SPEED_LPF_FREQ = 20.0f; // 转速低通滤波参数【单位：Hz】

    // 电机参数
    const float CURRENT_MAX; // 最大电流【单位；A】
    const uint16_t CAN_CMD_MAX; // CAN控制报文最大值
    const float REDUCTION_RATIO; // 电机减速比

    // 从CAN获得的电机数据
    struct motor_data_t {
        uint16_t ecd = 0; // 转子角度（减速前）【单位：0~8191 -> 0~360°】
        int16_t speed_rpm = 0; // 转速（减速前）【单位：rpm】
        int16_t current = 0; // 电流原始值
        uint8_t temperate = 0;  // 电机温度【单位：℃】
    } motor_data{};

    float can_recv_freq = 0; // CAN报文接收频率【单位：Hz】

    // 作用：计算CAN报文接收频率、计算加速度、速度加速度滤波
    uint32_t dwt_cnt = 0;
    float dt = 0;
};
