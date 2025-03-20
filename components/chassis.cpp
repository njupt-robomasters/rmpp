#include "chassis.hpp"
#include <cmath>
#include "arm_math.h"
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

void Chassis::SetSpeed(const float vx_set_mps, const float vy_set_mps, const float vr_set_tps) {
    this->vx_set_mps = vx_set_mps;
    this->vy_set_mps = vy_set_mps;
    this->vr_set_tps = vr_set_tps;
    vz_set_mps = vr_set_tps * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】
}

void Chassis::SetForwardAngle(const float forward_angle) {
    this->forward_angle = forward_angle;
}

void Chassis::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 设置电机转速目标值
        m1.SetV(v1_mps_set / WHEEL_PERIMETER, v1_a_ff / 2 / static_cast<float>(M_PI));
        m2.SetV(v2_mps_set / WHEEL_PERIMETER, v2_a_ff / 2 / static_cast<float>(M_PI));
        m3.SetV(v3_mps_set / WHEEL_PERIMETER, v3_a_ff / 2 / static_cast<float>(M_PI));
        m4.SetV(v4_mps_set / WHEEL_PERIMETER, v4_a_ff / 2 / static_cast<float>(M_PI));
    }

    // 电机闭环控制计算（即使使能也要调用，防止dt计算出现问题）
    m1.Update();
    m2.Update();
    m3.Update();
    m4.Update();

    // 发送CAN报文
    sendCurrentCMD();
}

void Chassis::forwardCalc() {
    // 根据设置的正方向，旋转x y速度分量
    vx_rotate_set_mps = vx_set_mps;
    vy_rotate_set_mps = vy_set_mps;
    rotate(vx_rotate_set_mps, vy_rotate_set_mps, forward_angle);

    // 根据速度分量，计算电机目标转速（全向轮运动学）
    // 前进方向为y轴正方向，右平移方向为x轴正方向，一 二 三 四 象限分别为 1 2 3 4 电机
    // 所有电机正转，底盘顺时针旋转（但规定逆时针为底盘旋转正方向）
    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    v1_mps_set = +sqrt2div2 * vx_rotate_set_mps - sqrt2div2 * vy_rotate_set_mps - vz_set_mps; // 轮子线速度【单位：m/s】
    v2_mps_set = +sqrt2div2 * vx_rotate_set_mps + sqrt2div2 * vy_rotate_set_mps - vz_set_mps; // 轮子线速度【单位：m/s】
    v3_mps_set = -sqrt2div2 * vx_rotate_set_mps + sqrt2div2 * vy_rotate_set_mps - vz_set_mps; // 轮子线速度【单位：m/s】
    v4_mps_set = -sqrt2div2 * vx_rotate_set_mps - sqrt2div2 * vy_rotate_set_mps - vz_set_mps; // 轮子线速度【单位：m/s】

    const float w = vr_set_tps * 2 * static_cast<float>(M_PI);
    const float x = vx_set_mps;
    const float y = vy_set_mps;
    const float a = forward_angle / 180.0f * static_cast<float>(M_PI);
    d_vx_rotate = - (- w * x * arm_sin_f32(a) - w * y * arm_cos_f32(a));
    d_vy_rotate = - (+ w * x * arm_cos_f32(a) - w * y * arm_sin_f32(a));
    v1_a_ff = +sqrt2div2 * d_vx_rotate - sqrt2div2 * d_vy_rotate;
    v2_a_ff = +sqrt2div2 * d_vx_rotate + sqrt2div2 * d_vy_rotate;
    v3_a_ff = -sqrt2div2 * d_vx_rotate + sqrt2div2 * d_vy_rotate;
    v4_a_ff = -sqrt2div2 * d_vx_rotate - sqrt2div2 * d_vy_rotate;
}

void Chassis::inverseCalc() {
    // 轮子角速度 -> 线速度
    v1_mps = m1.v_measure_lpf_tps * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v2_mps = m2.v_measure_lpf_tps * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v3_mps = m3.v_measure_lpf_tps * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】
    v4_mps = m4.v_measure_lpf_tps * WHEEL_PERIMETER; // 轮子线速度【单位：m/s】

    // 逆运动学解算当前底盘实际速度
    constexpr float sqrt2 = sqrtf(2);
    vx_rotate_measure_mps = (+sqrt2 * v1_mps + sqrt2 * v2_mps - sqrt2 * v3_mps - sqrt2 * v4_mps) / 4.0f;
    vy_rotate_measure_mps = (-sqrt2 * v1_mps + sqrt2 * v2_mps + sqrt2 * v3_mps - sqrt2 * v4_mps) / 4.0f;
    vz_measure_mps = -(v1_mps + v2_mps + v3_mps + v4_mps) / 4.0f;
    vr_measure_tps = vz_measure_mps / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】

    // 根据设置的正方向，旋转x y速度分量
    vx_measure_mps = vx_rotate_measure_mps;
    vy_measure_mps = vy_rotate_measure_mps;
    rotate(vx_measure_mps, vy_measure_mps, -forward_angle);
}

void Chassis::sendCurrentCMD() const {
    const int16_t m3508_1_cmd = m1.GetCurrentCMD();
    const int16_t m3508_2_cmd = m2.GetCurrentCMD();
    const int16_t m3508_3_cmd = m3.GetCurrentCMD();
    const int16_t m3508_4_cmd = m4.GetCurrentCMD();

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
