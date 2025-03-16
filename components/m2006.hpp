#pragma once

#include <cstdint>
#include "pid.hpp"

class M2006 {
public:
    static constexpr auto MAX_CURRENT = 10.0f; // 最大电流【单位；A】（CAN通信最大支持10A）
    static constexpr auto REDUCTION_RATIO = 36.0f; // 减速比

    // 从CAN获得的参数
    uint16_t ecd = 0; // 转子机械角度（减速前）【单位：0~8191 -> 0~360°】
    float speed_rpm = 0; // 减速箱输出轴转速（减速后）【单位：rpm】
    float current = 0; // 电流【单位：A】
    uint8_t temperate = 0; // 电机温度【单位：℃】

    float speed_rpm_set = 0; // 目标速度（减速后）【单位：rpm】
    float current_set = 0; // PID计算得出的电流【单位：A】

    void Init(float Kp, float Ki, float Kd, float max_Iout);

    void ParseCAN(const uint8_t data[8]);

    void Update(float speed_rpm_);

    void Release();

    int16_t GetCurrentCmd();

private:
    PID pid{};
};
