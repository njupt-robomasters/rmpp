#pragma once

#include <cstdint>
#include "pid.hpp"

class M6020 {
public:
    static constexpr auto MAX_CURRENT = 3.0f; // 最大电流【单位；A】（CAN通信最大支持3A）

    // 从CAN获得的参数
    uint16_t ecd = 0; // 转子机械角度【单位：0~8191 -> 0~360°】
    float speed_rpm = 0; // 转速【单位：rpm】
    float current = 0; // 电流【单位：A】
    uint8_t temperate = 0; // 电机温度【单位：℃】

    float angle = 0; // 当前角度【单位：度】
    float angle_set = 0; // 目标角度【单位：度】
    float current_set = 0; // PID计算得出的电流【单位：A】

    void Init(float Kp, float Ki, float Kd, float I_limit);

    void ParseCAN(const uint8_t data[8]);

    void Update(float angle_set);

    void Release();

    int16_t GetCANCmd();

private:
    PID pid{};

    float normAngle(float angle);

    float calcAngleErr(float now, float set);
};
