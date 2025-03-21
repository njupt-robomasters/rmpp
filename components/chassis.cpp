#include "chassis.hpp"
#include <cmath>
#include "bsp_can.h"
#include "utils.hpp"

Chassis::Chassis(const PID::pid_param_t &wheel_pid_param,
                 const PID::pid_param_t &chassis_pid_param) : vx_comp_pid(chassis_pid_param, SPEED_COMP_MAX),
                                                              vy_comp_pid(chassis_pid_param, SPEED_COMP_MAX),
                                                              m1(wheel_pid_param),
                                                              m2(wheel_pid_param),
                                                              m3(wheel_pid_param),
                                                              m4(wheel_pid_param) {
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

void Chassis::ResetReady() {
    m1.ResetReady();
    m2.ResetReady();
    m3.ResetReady();
    m4.ResetReady();
}

bool Chassis::CheckReady() const {
    if (not m1.IsReady())
        return false;
    if (not m2.IsReady())
        return false;
    if (not m3.IsReady())
        return false;
    if (not m4.IsReady())
        return false;
    return true;
}

void Chassis::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
    m1.SetEnable(is_enable);
    m2.SetEnable(is_enable);
    m3.SetEnable(is_enable);
    m4.SetEnable(is_enable);
}

void Chassis::SetSpeed(const float vx, const float vy, const float vr_rpm) {
    ref.vx.gimbal = vx;
    ref.vy.gimbal = vy;
    ref.vr.rpm = vr_rpm;
    ref.vz = ref.vr.tps * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】
}

void Chassis::SetGimbalRefByChassisAngle(const Angle &gimbal_ref_by_chassis) {
    this->gimbal_ref_by_chassis = gimbal_ref_by_chassis;
}

void Chassis::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 设置电机转速目标值
        Speed speed1, speed2, speed3, speed4;
        speed1.tps = ref.v1 / WHEEL_PERIMETER;
        speed2.tps = ref.v2 / WHEEL_PERIMETER;
        speed3.tps = ref.v3 / WHEEL_PERIMETER;
        speed4.tps = ref.v4 / WHEEL_PERIMETER;
        m1.SetSpeed(speed1);
        m2.SetSpeed(speed2);
        m3.SetSpeed(speed3);
        m4.SetSpeed(speed4);
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
    const float vx_err = ref.vx.gimbal - measure.vx.gimbal;
    vx_comp_pid.CalcIncrement(vx_err);

    const float vy_err = ref.vy.gimbal - measure.vy.gimbal;
    vy_comp_pid.CalcIncrement(vy_err);
}

void Chassis::forwardCalc() {
    // 计算速度补偿
    calcSpeedComp();

    // vx vy 换算到底盘坐标系
    // 注意这里是换参考系，而非旋转速度矢量，速度矢量相对于绝对参考系是不会发生变化的，所以旋转角度为：云台相对于底盘的角度
    std::tie(ref.vx.chassis, ref.vy.chassis) = rotate(ref.vx.gimbal + vx_comp, ref.vy.gimbal + vy_comp,
                                                      gimbal_ref_by_chassis.degree);

    // 根据速度分量，计算轮子线速度【单位：m/s】
    // 前进方向为y轴正方向，右平移方向为x轴正方向，一 二 三 四 象限分别为 1 2 3 4 电机
    // 所有电机正转，底盘顺时针旋转（但规定逆时针为底盘旋转正方向）
    constexpr float sqrt2div2 = sqrtf(2) / 2.0f;
    ref.v1 = +sqrt2div2 * ref.vx.chassis - sqrt2div2 * ref.vy.chassis - ref.vz;
    ref.v2 = +sqrt2div2 * ref.vx.chassis + sqrt2div2 * ref.vy.chassis - ref.vz;
    ref.v3 = -sqrt2div2 * ref.vx.chassis + sqrt2div2 * ref.vy.chassis - ref.vz;
    ref.v4 = -sqrt2div2 * ref.vx.chassis - sqrt2div2 * ref.vy.chassis - ref.vz;
}

void Chassis::inverseCalc() {
    // 轮子角速度【圈/s】 -> 线速度【单位：m/s】
    measure.v1 = m1.measure.speed.tps * WHEEL_PERIMETER;
    measure.v2 = m2.measure.speed.tps * WHEEL_PERIMETER;
    measure.v3 = m3.measure.speed.tps * WHEEL_PERIMETER;
    measure.v4 = m4.measure.speed.tps * WHEEL_PERIMETER;

    // 逆运动学解算当前底盘实际速度分量
    constexpr float sqrt2 = sqrtf(2);
    measure.vx.chassis = (+sqrt2 * measure.v1 + sqrt2 * measure.v2 - sqrt2 * measure.v3 - sqrt2 * measure.v4) / 4.0f;
    measure.vy.chassis = (-sqrt2 * measure.v1 + sqrt2 * measure.v2 + sqrt2 * measure.v3 - sqrt2 * measure.v4) / 4.0f;
    measure.vz = -(measure.v1 + measure.v2 + measure.v3 + measure.v4) / 4.0f;
    measure.vr.tps = measure.vz / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】

    // vx vy 换算到云台坐标系
    std::tie(ref.vx.gimbal, ref.vy.gimbal) = rotate(measure.vx.chassis, measure.vy.chassis,
                                                    -gimbal_ref_by_chassis.degree);
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
