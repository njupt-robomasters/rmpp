#include "chassis_steer.hpp"
#include "bsp_can.h"

ChassisSteer::ChassisSteer(PID::param_t &servo_pid_param, PID::param_t &wheel_pid_param)
    : s1(servo_pid_param),
      s2(servo_pid_param),
      w1(wheel_pid_param),
      w2(wheel_pid_param) {
    // 舵轮默认前进方向
    ref.wheel.s1.relative.degree = 90;
    ref.wheel.s2.relative.degree = 90;
}

void ChassisSteer::ParseCAN(const uint32_t id, uint8_t data[8]) {
    if (id == 0x205)
        s1.ParseCAN(data); // 右前舵电机，6020
    if (id == 0x206)
        s2.ParseCAN(data); // 左后舵电机，6020
    if (id == 0x207)
        w1.ParseCAN(data); // 右前轮电机，3508
    if (id == 0x208)
        w2.ParseCAN(data); // 左后轮电机，3508

    inverseCalc(); // 运动学逆解
    estimatePower(); // 底盘功率估算
}

void ChassisSteer::ResetReady() {
    s1.ResetReady();
    s2.ResetReady();
    w1.ResetReady();
    w2.ResetReady();
}

bool ChassisSteer::CheckReady() const {
    if (not s1.IsReady())
        return false;
    if (not s2.IsReady())
        return false;
    if (not w1.IsReady())
        return false;
    if (not w2.IsReady())
        return false;
    return true;
}

void ChassisSteer::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;

    this->is_enable = is_enable;

    // 设置子设备
    s1.SetEnable(is_enable);
    s2.SetEnable(is_enable);
    w1.SetEnable(is_enable);
    w2.SetEnable(is_enable);
}

void ChassisSteer::SetSpeed(const float vx, const float vy, const float vr_rpm) {
    ref.chassis.vx = vx;
    ref.chassis.vy = vy;
    ref.chassis.vr.rpm = vr_rpm;
}

void ChassisSteer::SetPowerLimit(const float power) {
    power_limit = power;
}

void ChassisSteer::Update() {
    if (is_enable) {
        // 运动学正解
        forwardCalc();

        // 电机闭环控制计算
        s1.Update();
        s2.Update();
        w1.Update();
        w2.Update();

        // 功率控制
        calcCurrentRatio();
    }

    // 发送CAN报文
    sendCANCmd();
}

// 估算底盘当前功率，用于调试
void ChassisSteer::estimatePower() {
    const float power1 = w1.EstimatePower();
    const float power2 = w2.EstimatePower();
    power_estimate = power1 + power2;
}

// 计算电流衰减系数，需要在电机PID计算后调用
void ChassisSteer::calcCurrentRatio() {
    // Mw + I^2*R = P
    // kIw + I^2*R = P
    // M_PER_I * xI * w + (xI)^2 * R = P
    // I^2*R * x^2 + M_PER_I*I*w * x - P = 0
    // a = I^2*R
    // b = M_PER_I*I*w
    // c = -P

    float a = 0;
    a += w1.ref.current * w1.ref.current * M3508::R;
    a += w2.ref.current * w2.ref.current * M3508::R;

    float b = 0;
    b += M3508::M_PER_I * w1.ref.current * w1.measure.speed.rps;
    b += M3508::M_PER_I * w2.ref.current * w2.measure.speed.rps;

    float c = -power_limit;

    current_ratio = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);

    current_ratio = clamp(current_ratio, 0, 1);
}

// 运动学正解
void ChassisSteer::forwardCalc() {
    ref.chassis.vz = ref.chassis.vr.tps * CHASSIS_PERIMETER; // 底盘旋转角速度【圈/s】 -> 底盘旋转线速度【m/s】

    // 1. 运动学正解
    // 舵电机正方向：逆时针
    // 轮电机正方向：车体前进方向
    const float sqrt2div2 = sqrtf(2) / 2;
    float y, x;
    // 左前舵、左前轮（1号）
    y = ref.chassis.vy - sqrt2div2 * ref.chassis.vz;
    x = ref.chassis.vx - sqrt2div2 * ref.chassis.vz;
    if (fabsf(x) > ESP || fabsf(y) > ESP) {
        ref.wheel.s1.relative.rad = atan2f(y, x);
    }
    ref.wheel.w1 = sqrtf(powf(y, 2) + powf(x, 2));
    // 右后舵、右后轮（2号）
    y = ref.chassis.vy + sqrt2div2 * ref.chassis.vz;
    x = ref.chassis.vx + sqrt2div2 * ref.chassis.vz;
    if (fabsf(x) > ESP || fabsf(y) > ESP) {
        ref.wheel.s2.relative.rad = atan2f(y, x);
    }
    ref.wheel.w2 = sqrtf(powf(y, 2) + powf(x, 2));

    // 2. 舵电机安装矫正
    // 目前6020为顺时针角度增加，所以要取反
    ref.wheel.s1.absolute.degree = norm_angle( -ref.wheel.s1.relative.degree + (S1_OFFSET + 90));
    ref.wheel.s2.absolute.degree = norm_angle(-ref.wheel.s2.relative.degree + (S2_OFFSET + 90));

    // 3. 就近转向
    // 1号轮
    float s1_angle_err = calc_angle_err(ref.wheel.s1.absolute.degree, measure.wheel.s1.absolute.degree);
    if (s1_angle_err < -90.0f) {
        s1_angle_err += 180.0f;
        ref.wheel.w1 = -ref.wheel.w1;
    } else if (s1_angle_err > 90.0f) {
        s1_angle_err -= 180.0f;
        ref.wheel.w1 = -ref.wheel.w1;
    }
    ref.wheel.s1.absolute.degree = norm_angle(measure.wheel.s1.absolute.degree + s1_angle_err);
    // 2号轮
    float s2_angle_err = calc_angle_err(ref.wheel.s2.absolute.degree, measure.wheel.s2.absolute.degree);
    if (s2_angle_err < -90.0f) {
        s2_angle_err += 180.0f;
        ref.wheel.w2 = -ref.wheel.w2;
    } else if (s2_angle_err > 90.0f) {
        s2_angle_err -= 180.0f;
        ref.wheel.w2 = -ref.wheel.w2;
    }
    ref.wheel.s2.absolute.degree = norm_angle(measure.wheel.s2.absolute.degree + s2_angle_err);

    // 4. 轮子线速度 -> 轮子角速度
    Speed speed1, speed2;
    speed1.tps = ref.wheel.w1 / WHEEL_PERIMETER;
    speed2.tps = ref.wheel.w2 / WHEEL_PERIMETER;

    // 5. 设置电机
    s1.SetAngle(ref.wheel.s1.absolute);
    s2.SetAngle(ref.wheel.s2.absolute);
    w1.SetSpeed(speed1);
    w2.SetSpeed(speed2);
}

// 运动学逆解
void ChassisSteer::inverseCalc() {
    // todo 逆解算没有考虑到就近转向问题

    // 1. 读取电机
    measure.wheel.s1.absolute = s1.measure.angle;
    measure.wheel.s2.absolute = s2.measure.angle;
    const Speed speed1 = w1.measure.speed;
    const Speed speed2 = w2.measure.speed;

    // 2. 轮子角速度 -> 轮子线速度
    measure.wheel.w1 = speed1.tps * WHEEL_PERIMETER;
    measure.wheel.w2 = speed2.tps * WHEEL_PERIMETER;

    // 3. 舵电机安装逆解
    measure.wheel.s1.relative.degree = norm_angle( -(measure.wheel.s1.absolute.degree - (S1_OFFSET + 90)));
    measure.wheel.s2.relative.degree = norm_angle(-(measure.wheel.s2.absolute.degree - (S2_OFFSET + 90)));

    // 4. 运动学逆解
    float y1 = measure.wheel.w1 * sinf(measure.wheel.s1.relative.rad);
    float x1 = measure.wheel.w1 * cosf(measure.wheel.s1.relative.rad);
    float y2 = measure.wheel.w2 * sinf(measure.wheel.s2.relative.rad);
    float x2 = measure.wheel.w2 * cosf(measure.wheel.s2.relative.rad);
    measure.chassis.vy = (y1 + y2) / 2;
    measure.chassis.vx = (x1 + x2) / 2;
    measure.chassis.vz = (measure.wheel.w1 - measure.wheel.w2) / 2;

    measure.chassis.vr.tps = measure.chassis.vz / CHASSIS_PERIMETER; // 底盘旋转线速度【m/s】 -> 底盘旋转角速度【圈/s】
}

void ChassisSteer::sendCANCmd() {
    const int16_t m6020_1_cmd = s1.GetCurrentCMD();
    const int16_t m6020_2_cmd = s2.GetCurrentCMD();
    const int16_t m3508_1_cmd = w1.GetCurrentCMD() * current_ratio;
    const int16_t m3508_2_cmd = w2.GetCurrentCMD() * current_ratio;


    uint8_t data[8];
    data[0] = m6020_1_cmd >> 8; // 6020电机，ID：5
    data[1] = m6020_1_cmd;
    data[2] = m6020_2_cmd >> 8; // 6020电机，ID：6
    data[3] = m6020_2_cmd;
    data[4] = m3508_1_cmd >> 8; // 3508电机，ID：7
    data[5] = m3508_1_cmd;
    data[6] = m3508_2_cmd >> 8; // 3508电机，ID：8
    data[7] = m3508_2_cmd;

    BSP_CAN_Transmit(0x1FF, data, 8);
}
