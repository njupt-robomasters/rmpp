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

void Chassis::SetSpeed(const float vx_ref, const float vy_ref, const float vr_tps_ref) {
    vx_gimbal.ref = vx_ref;
    vy_gimbal.ref = vy_ref;
    vr_tps.ref = vr_tps_ref;
    vz.ref = vr_tps.ref * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】
}

void Chassis::SetGimbalRefByChassisAngle(const float gimbal_ref_by_chassis) {
    this->gimbal_ref_by_chassis = gimbal_ref_by_chassis;
}

void Chassis::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 设置电机转速目标值
        m1.SetV(v1.ref / WHEEL_PERIMETER);
        m2.SetV(v2.ref / WHEEL_PERIMETER);
        m3.SetV(v3.ref / WHEEL_PERIMETER);
        m4.SetV(v4.ref / WHEEL_PERIMETER);
    }

    // 电机闭环控制计算（即使未使能也要调用，防止dt计算出现问题）
    m1.Update();
    m2.Update();
    m3.Update();
    m4.Update();

    // 发送CAN报文
    sendCurrentCMD();
}

void Chassis::calcSpeedComp() {
    const float vx_err = vx_gimbal.ref - vx_gimbal.measure;
    vx_comp = kp * vx_err;

    const float vy_err = vy_gimbal.ref - vy_gimbal.measure;
    vy_comp = kp * vy_err;
}

void Chassis::forwardCalc() {
    // 补偿速度计算
    calcSpeedComp();

    // 根据底盘与云台的夹角，旋转x y速度分量
    vx_chassis.ref = vx_gimbal.ref + vx_comp;
    vy_chassis.ref = vy_gimbal.ref + vy_comp;
    // 注意这里是换参考系，而非旋转速度矢量，速度矢量相对于绝对参考系是不会发生变化的
    rotate(vx_chassis.ref, vy_chassis.ref, gimbal_ref_by_chassis);

    // 根据速度分量，计算电机目标转速（全向轮运动学）
    // 前进方向为y轴正方向，右平移方向为x轴正方向，一 二 三 四 象限分别为 1 2 3 4 电机
    // 所有电机正转，底盘顺时针旋转（但规定逆时针为底盘旋转正方向）
    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    v1.ref = +sqrt2div2 * vx_chassis.ref - sqrt2div2 * vy_chassis.ref - vz.ref; // 轮子线速度【单位：m/s】
    v2.ref = +sqrt2div2 * vx_chassis.ref + sqrt2div2 * vy_chassis.ref - vz.ref; // 轮子线速度【单位：m/s】
    v3.ref = -sqrt2div2 * vx_chassis.ref + sqrt2div2 * vy_chassis.ref - vz.ref; // 轮子线速度【单位：m/s】
    v4.ref = -sqrt2div2 * vx_chassis.ref - sqrt2div2 * vy_chassis.ref - vz.ref; // 轮子线速度【单位：m/s】
}

void Chassis::inverseCalc() {
    // 轮子角速度【圈/s】 -> 线速度【单位：m/s】
    v1.measure = m1.v_measure_lpf_tps * WHEEL_PERIMETER;
    v2.measure = m2.v_measure_lpf_tps * WHEEL_PERIMETER;
    v3.measure = m3.v_measure_lpf_tps * WHEEL_PERIMETER;
    v4.measure = m4.v_measure_lpf_tps * WHEEL_PERIMETER;

    // 逆运动学解算当前底盘实际速度
    constexpr float sqrt2 = sqrtf(2);
    vx_chassis.measure = (+sqrt2 * v1.measure + sqrt2 * v2.measure - sqrt2 * v3.measure - sqrt2 * v4.measure) / 4.0f;
    vy_chassis.measure = (-sqrt2 * v1.measure + sqrt2 * v2.measure + sqrt2 * v3.measure - sqrt2 * v4.measure) / 4.0f;
    vz.measure = -(v1.measure + v2.measure + v3.measure + v4.measure) / 4.0f;
    vr_tps.measure = vz.measure / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】

    // 根据设置的正方向，旋转x y速度分量
    vx_gimbal.measure = vx_chassis.measure;
    vy_gimbal.measure = vy_chassis.measure;
    rotate(vx_gimbal.measure, vy_gimbal.measure, -gimbal_ref_by_chassis);
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
