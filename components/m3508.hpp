#pragma once

#include <cstdint>
#include "pid.hpp"

class M3508 {
public:
    static constexpr float MAX_CURRENT = 20.0f; // 最大电流【单位；A】（CAN通信最大支持20A）
    static constexpr float REDUCTION_RATIO = 3591.0f / 187.0f; // 减速比

    // 从CAN获得的参数
    uint16_t ecd = 0; // 转子机械角度（减速前）【单位：0~8191 -> 0~360°】
    float speed_rpm = 0; // 减速箱输出轴转速（减速后）【单位：rpm】
    float current = 0; // 电流【单位：A】
    uint8_t temperate = 0; // 电机温度【单位：℃】

    float speed_rpm_set = 0; // 目标速度（减速后）【单位：rpm】
    float current_set = 0; // PID计算得出的电流【单位：A】

    void Init(float Kp, float Ki, float Kd, float I_limit);

    void ParseCAN(const uint8_t data[8]);

    void Update(float speed_rpm_set);

    void Release();

    int16_t GetCANCmd();

private:
    PID speed_pid{};
};
