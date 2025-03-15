#include "chassis.hpp"
#include <cmath>
#include "bsp_can.h"

void Chassis::Init() {
    m1.Init(M3508_KP, M3508_KI, M3508_KD, M3508_MAX_IOUT);
    m2.Init(M3508_KP, M3508_KI, M3508_KD, M3508_MAX_IOUT);
    m3.Init(M3508_KP, M3508_KI, M3508_KD, M3508_MAX_IOUT);
    m4.Init(M3508_KP, M3508_KI, M3508_KD, M3508_MAX_IOUT);
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
}

void Chassis::Update(const float vx_, const float vy_, const float vr_) {
    vx_set = vx_;
    vy_set = vy_;
    vr_set = vr_;
    vz_set = vr_set / 60.0f * CHASSIS_PERIMETER; // 底盘旋转角速度rpm -> 底盘旋转线速度m/s

    forwardCalc();
    inverseCalc();
    sendCANCmd();
}

// 释放底盘，关闭所有底盘电机动力输出
void Chassis::Release() {
    m1.Release();
    m2.Release();
    m3.Release();
    m4.Release();

    sendCANCmd();

    // 仍然维持底盘逆解算
    inverseCalc();

    // 清空所有设定值
    vx_set = vy_set = vr_set = vz_set = 0;
    v1_set = v2_set = v3_set = v4_set = 0;
}

void Chassis::forwardCalc() {
    // 根据速度分量，计算电机目标转速（全向轮运动学解算）
    constexpr auto sqrt2div2 = sqrtf(2) / 2.0f;
    v1_set = -sqrt2div2 * vx_set + sqrt2div2 * vy_set + vz_set; // 轮子线速度【单位：m/s】
    v2_set = -sqrt2div2 * vx_set - sqrt2div2 * vy_set + vz_set; // 轮子线速度【单位：m/s】
    v3_set = sqrt2div2 * vx_set - sqrt2div2 * vy_set + vz_set; // 轮子线速度【单位：m/s】
    v4_set = sqrt2div2 * vx_set + sqrt2div2 * vy_set + vz_set; // 轮子线速度【单位：m/s】

    m1.Update(v1_set / WHEEL_PERIMETER * 60.0f);
    m2.Update(v2_set / WHEEL_PERIMETER * 60.0f);
    m3.Update(v3_set / WHEEL_PERIMETER * 60.0f);
    m4.Update(v4_set / WHEEL_PERIMETER * 60.0f);
}

void Chassis::inverseCalc() {
    // 逆运动学解算当前底盘实际速度
    const float v1 = m1.speed_rpm / 60.0f * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    const float v2 = m2.speed_rpm / 60.0f * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    const float v3 = m3.speed_rpm / 60.0f * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    const float v4 = m4.speed_rpm / 60.0f * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】

    constexpr auto sqrt2div2 = sqrtf(2) / 2.0f;
    vx = -sqrt2div2 * v1 - sqrt2div2 * v2 + sqrt2div2 * v3 + sqrt2div2 * v4;
    vy = sqrt2div2 * v1 - sqrt2div2 * v2 - sqrt2div2 * v3 + sqrt2div2 * v4;
    vz = (v1 + v2 + v3 + v4) / 4.0f;
    vr = vz / CHASSIS_PERIMETER * 60.0f; // 底盘旋转线速度m/s -> 底盘旋转角速度rpm
}

void Chassis::sendCANCmd() {
    const int16_t m3508_1_cmd = m1.GetCurrentCmd();
    const int16_t m3508_2_cmd = m2.GetCurrentCmd();
    const int16_t m3508_3_cmd = m3.GetCurrentCmd();
    const int16_t m3508_4_cmd = m4.GetCurrentCmd();

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
