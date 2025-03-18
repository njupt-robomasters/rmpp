#include "chassis.hpp"
#include <cmath>
#include "bsp_can.h"

Chassis::Chassis() : m1(KP, KD),
                     m2(KP, KD),
                     m3(KP, KD),
                     m4(KP, KD) {
}

void Chassis::ParseCAN(const uint32_t id, uint8_t data[8]) {
    if (id == 0x201)
        m1.ParseCAN(data);
    if (id == 0x202)
        m2.ParseCAN(data);
    if (id == 0x203)
        m3.ParseCAN(data);
    if (id == 0x204)
        m4.ParseCAN(data);

    inverseCalc();
}

void Chassis::Update(const float vx_set, const float vy_set, const float vr_set) {
    this->vx_set = vx_set;
    this->vy_set = vy_set;
    this->vr_set = vr_set;
    vz_set = vr_set / 60.0f * CHASSIS_PERIMETER; // 底盘旋转角速度rpm -> 底盘旋转线速度m/s

    forwardCalc();

    // 电机PID计算
    m1.Update(v1_set / WHEEL_PERIMETER);
    m2.Update(v2_set / WHEEL_PERIMETER);
    m3.Update(v3_set / WHEEL_PERIMETER);
    m4.Update(v4_set / WHEEL_PERIMETER);

    sendCANCmd();
}

// 释放底盘，关闭所有底盘电机动力输出
void Chassis::Release() {
    m1.Release();
    m2.Release();
    m3.Release();
    m4.Release();

    sendCANCmd();
}

void Chassis::forwardCalc() {
    // 根据速度分量，计算电机目标转速（全向轮运动学解算）
    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    v1_set = sqrt2div2 * vx_set - sqrt2div2 * vy_set - vz_set; // 轮子线速度【单位：m/s】
    v2_set = sqrt2div2 * vx_set + sqrt2div2 * vy_set - vz_set; // 轮子线速度【单位：m/s】
    v3_set = -sqrt2div2 * vx_set + sqrt2div2 * vy_set - vz_set; // 轮子线速度【单位：m/s】
    v4_set = -sqrt2div2 * vx_set - sqrt2div2 * vy_set - vz_set; // 轮子线速度【单位：m/s】
}

void Chassis::inverseCalc() {
    // 逆运动学解算当前底盘实际速度
    v1 = m1.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v2 = m2.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v3 = m3.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v4 = m4.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】

    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    vx = sqrt2div2 * v1 + sqrt2div2 * v2 - sqrt2div2 * v3 - sqrt2div2 * v4;
    vy = -sqrt2div2 * v1 + sqrt2div2 * v2 + sqrt2div2 * v3 - sqrt2div2 * v4;
    vz = -(v1 + v2 + v3 + v4) / 4.0f;
    vr = vz / CHASSIS_PERIMETER * 60.0f; // 底盘旋转线速度m/s -> 底盘旋转角速度rpm
}

void Chassis::sendCANCmd() const {
    const int16_t m3508_1_cmd = m1.GetCANCmd();
    const int16_t m3508_2_cmd = m2.GetCANCmd();
    const int16_t m3508_3_cmd = m3.GetCANCmd();
    const int16_t m3508_4_cmd = m4.GetCANCmd();

    uint8_t data[8];
    data[0] = m3508_1_cmd >> 8; // 3508电机，ID：1
    data[1] = m3508_1_cmd;
    data[2] = m3508_2_cmd >> 8; // 3508电机，ID：2
    data[3] = m3508_2_cmd;
    data[4] = m3508_3_cmd >> 8; // 3508电机，ID：3
    data[5] = m3508_3_cmd;
    data[6] = m3508_4_cmd >> 8; // 3508电机，ID：4
    data[7] = m3508_4_cmd;

    BSP_CAN_Transmit(0x200, data, 8);
}
