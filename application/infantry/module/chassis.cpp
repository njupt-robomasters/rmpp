#include "chassis.hpp"
#include "bsp_can.h"

Chassis::Chassis(PID::param_t &wheel_pid_param): m1(wheel_pid_param),
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

    inverseCalc(); // 运动学逆解
    estimatePower(); // 底盘功率估算
}

void Chassis::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;

    // 设置子设备
    m1.SetEnable(is_enable);
    m2.SetEnable(is_enable);
    m3.SetEnable(is_enable);
    m4.SetEnable(is_enable);
}

void Chassis::SetSpeed(const float vx, const float vy, const float vr_rpm) {
    ref.gimbal.vx = vx;
    ref.gimbal.vy = vy;
    ref.gimbal.vr.rpm = vr_rpm;
}

void Chassis::SetGimbalAngle_RefByChassis(const Angle &gimbal_angle_refBy_chassis) {
    this->gimbal_angle_refBy_chassis = gimbal_angle_refBy_chassis;
}

void Chassis::SetPowerLimit(const float power) {
    power_limit = power;
}

void Chassis::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 电机闭环控制计算
        m1.Update();
        m2.Update();
        m3.Update();
        m4.Update();

        // 功率控制
        calcCurrentRatio();
    }

    // 发送CAN报文
    sendCurrentCmd();
}

// 运动学正解
void Chassis::forwardCalc() {
    ref.gimbal.vz = ref.gimbal.vr.tps * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】

    // 1. vxyzr 云台坐标系->底盘坐标系
    // 注意这里是换参考系，而非旋转速度矢量，速度矢量相对于绝对参考系是不会发生变化的，所以旋转角度为：云台相对于底盘的角度
    std::tie(ref.chassis.vx, ref.chassis.vy) =
            rotate(ref.gimbal.vx, ref.gimbal.vy, gimbal_angle_refBy_chassis.degree);
    ref.chassis.vz = ref.gimbal.vz;
    ref.chassis.vr = ref.gimbal.vr;

    // 2. vxyzr -> 轮子线速度
    // 前进方向为y轴正方向，右平移方向为x轴正方向，一 二 三 四 象限分别为 1 2 3 4 电机
    const float sqrt2div2 = sqrtf(2) / 2.0f;
    // 所有电机正转，底盘顺时针旋转（但规定逆时针为底盘旋转正方向）
    ref.wheel.v1 = +sqrt2div2 * ref.chassis.vx - sqrt2div2 * ref.chassis.vy - ref.chassis.vz;
    ref.wheel.v2 = +sqrt2div2 * ref.chassis.vx + sqrt2div2 * ref.chassis.vy - ref.chassis.vz;
    ref.wheel.v3 = -sqrt2div2 * ref.chassis.vx + sqrt2div2 * ref.chassis.vy - ref.chassis.vz;
    ref.wheel.v4 = -sqrt2div2 * ref.chassis.vx - sqrt2div2 * ref.chassis.vy - ref.chassis.vz;

    // 3. 轮子线速度 -> 轮子角速度
    Speed speed1, speed2, speed3, speed4;
    speed1.tps = ref.wheel.v1 / WHEEL_PERIMETER;
    speed2.tps = ref.wheel.v2 / WHEEL_PERIMETER;
    speed3.tps = ref.wheel.v3 / WHEEL_PERIMETER;
    speed4.tps = ref.wheel.v4 / WHEEL_PERIMETER;

    // 4. 设置电机转速
    m1.SetSpeed(speed1);
    m2.SetSpeed(speed2);
    m3.SetSpeed(speed3);
    m4.SetSpeed(speed4);
}

// 运动学逆解
void Chassis::inverseCalc() {
    // 1. 读取电机转速
    const Speed speed1 = m1.measure.speed;
    const Speed speed2 = m2.measure.speed;
    const Speed speed3 = m3.measure.speed;
    const Speed speed4 = m4.measure.speed;

    // 2. 轮子角速度 -> 轮子线速度
    measure.wheel.v1 = speed1.tps * WHEEL_PERIMETER;
    measure.wheel.v2 = speed2.tps * WHEEL_PERIMETER;
    measure.wheel.v3 = speed3.tps * WHEEL_PERIMETER;
    measure.wheel.v4 = speed4.tps * WHEEL_PERIMETER;

    // 3. 轮子线速度 -> vxyzr
    const float sqrt2 = sqrtf(2);
    // 全向轮
    measure.chassis.vx = (+sqrt2 * measure.wheel.v1 + sqrt2 * measure.wheel.v2 - sqrt2 * measure.wheel.v3 - sqrt2 *
                          measure.wheel.v4) / 4.0f;
    measure.chassis.vy = (-sqrt2 * measure.wheel.v1 + sqrt2 * measure.wheel.v2 + sqrt2 * measure.wheel.v3 - sqrt2 *
                          measure.wheel.v4) / 4.0f;
    measure.chassis.vz = -(measure.wheel.v1 + measure.wheel.v2 + measure.wheel.v3 + measure.wheel.v4) / 4.0f;
    measure.chassis.vr.tps = measure.chassis.vz / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】

    // 4. vxyzr 底盘坐标系->云台坐标系
    std::tie(measure.gimbal.vx, measure.gimbal.vy) = rotate(measure.chassis.vx, measure.chassis.vy,
                                                            -gimbal_angle_refBy_chassis.degree);
    measure.gimbal.vz = measure.chassis.vz;
    measure.gimbal.vr = ref.chassis.vr;
}

void Chassis::estimatePower() {
    const float power1 = m1.EstimatePower();
    const float power2 = m2.EstimatePower();
    const float power3 = m3.EstimatePower();
    const float power4 = m4.EstimatePower();
    power_estimate = power1 + power2 + power3 + power4;
}

void Chassis::calcCurrentRatio() {
    // Mw + I^2*R = P
    // kIw + I^2*R = P
    // M_PER_I * xI * w + (xI)^2 * R = P
    // I^2*R * x^2 + M_PER_I*I*w * x - P = 0
    // a = I^2*R
    // b = M_PER_I*I*w
    // c = -P

    float a = 0;
    a += m1.ref.current * m1.ref.current * M3508::R;
    a += m2.ref.current * m2.ref.current * M3508::R;
    a += m3.ref.current * m3.ref.current * M3508::R;
    a += m4.ref.current * m4.ref.current * M3508::R;

    float b = 0;
    b += M3508::M_PER_I * m1.ref.current * m1.measure.speed.rps;
    b += M3508::M_PER_I * m2.ref.current * m2.measure.speed.rps;
    b += M3508::M_PER_I * m3.ref.current * m3.measure.speed.rps;
    b += M3508::M_PER_I * m4.ref.current * m4.measure.speed.rps;

    float c = -power_limit;

    current_ratio = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);

    current_ratio = clamp(current_ratio, 0, 1);
}

void Chassis::sendCurrentCmd() const {
    const int16_t m3508_1_cmd = m1.GetCurrentCMD() * current_ratio;
    const int16_t m3508_2_cmd = m2.GetCurrentCMD() * current_ratio;
    const int16_t m3508_3_cmd = m3.GetCurrentCMD() * current_ratio;
    const int16_t m3508_4_cmd = m4.GetCurrentCMD() * current_ratio;

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
