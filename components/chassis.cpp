#include "chassis.hpp"
#include <cmath>
#include "bsp_can.h"
#include "utils.hpp"

Chassis::Chassis(const mit_t &mit) : m1(mit),
                                     m2(mit),
                                     m3(mit),
                                     m4(mit) {
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

    // 运动学逆解
    inverseCalc();
}

void Chassis::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
    m1.SetEnable(is_enable);
    m2.SetEnable(is_enable);
    m3.SetEnable(is_enable);
    m4.SetEnable(is_enable);
}

void Chassis::SetSpeed(const float vx_mps_set, const float vy_mps_set, const float vr_tps_set) {
    this->vx_mps_set = vx_mps_set;
    this->vy_mps_set = vy_mps_set;
    this->vr_tps_set = vr_tps_set;
    vz_mps_set = vr_tps_set * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】
}

void Chassis::SetForwardAngle(const float forward_angle) {
    this->forward_angle = forward_angle;
}

void Chassis::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 设置电机转速目标值
        m1.SetV_TPS(v1_mps_set / WHEEL_PERIMETER);
        m2.SetV_TPS(v2_mps_set / WHEEL_PERIMETER);
        m3.SetV_TPS(v3_mps_set / WHEEL_PERIMETER);
        m4.SetV_TPS(v4_mps_set / WHEEL_PERIMETER);
    }

    // 电机闭环控制计算（即使使能也要调用，防止dt计算出现问题）
    m1.Update();
    m2.Update();
    m3.Update();
    m4.Update();

    // 发送CAN报文
    sendCANCmd();
}

void Chassis::forwardCalc() {
    // 根据设置的正方向，旋转x y速度分量
    vx_mps_rotate_set = vx_mps_set;
    vy_mps_rotate_set = vy_mps_set;
    rotate(vx_mps_rotate_set, vy_mps_rotate_set, forward_angle);

    // 根据速度分量，计算电机目标转速（全向轮运动学）
    // 前进方向为y轴正方向，右平移方向为x轴正方向，一 二 三 四 象限分别为 1 2 3 4 电机
    // 所有电机正转，底盘顺时针旋转（但规定逆时针为底盘旋转正方向）
    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    v1_mps_set = sqrt2div2 * vx_mps_rotate_set - sqrt2div2 * vy_mps_rotate_set - vz_mps_set; // 轮子线速度【单位：m/s】
    v2_mps_set = sqrt2div2 * vx_mps_rotate_set + sqrt2div2 * vy_mps_rotate_set - vz_mps_set; // 轮子线速度【单位：m/s】
    v3_mps_set = -sqrt2div2 * vx_mps_rotate_set + sqrt2div2 * vy_mps_rotate_set - vz_mps_set; // 轮子线速度【单位：m/s】
    v4_mps_set = -sqrt2div2 * vx_mps_rotate_set - sqrt2div2 * vy_mps_rotate_set - vz_mps_set; // 轮子线速度【单位：m/s】
}

void Chassis::inverseCalc() {
    // 轮子角速度 -> 线速度
    v1_mps = m1.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v2_mps = m2.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v3_mps = m3.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v4_mps = m4.v_tps_lpf * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】

    // 逆运动学解算当前底盘实际速度
    constexpr float sqrt2 = sqrtf(2);
    vx_mps_rotate = (sqrt2 * v1_mps + sqrt2 * v2_mps - sqrt2 * v3_mps - sqrt2 * v4_mps) / 4.0f;
    vy_mps_rotate = (-sqrt2 * v1_mps + sqrt2 * v2_mps + sqrt2 * v3_mps - sqrt2 * v4_mps) / 4.0f;
    vz_mps = -(v1_mps + v2_mps + v3_mps + v4_mps) / 4.0f;
    vr_tps = vz_mps / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】

    // 根据设置的正方向，旋转x y速度分量
    vx_mps = vx_mps_rotate;
    vy_mps = vy_mps_rotate;
    rotate(vx_mps, vy_mps, -forward_angle);
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
