#include "chassis.hpp"
#include "app.hpp"
#include "utils.hpp"

Chassis::Chassis(PID::param_t* m6020_pid_param, PID::param_t* m3508_pid_param) :
    m6020_1(1, 0x207),
    m6020_2(1, 0x208),
    m3508_1(1, 0x201),
    m3508_2(1, 0x202) {
    // 设置PID参数
    m6020_1.SetPIDParam(m6020_pid_param);
    m6020_2.SetPIDParam(m6020_pid_param);
    m3508_1.SetPIDParam(m3508_pid_param);
    m3508_2.SetPIDParam(m3508_pid_param);

    // 设置减速比
    m3508_1.SetReduction(14.0f);
    m3508_2.SetReduction(14.0f);

    // 反转电机
    m6020_1.SetInvert(true);
    m6020_2.SetInvert(true);
}

// 运动学正解
void Chassis::forwardCalc() {
    // 1. 读取电机
    s1.absolute.measure = m6020_1.angle.measure;
    s2.absolute.measure = m6020_2.angle.measure;
    v1.measure = m3508_1.speed.measure * WHEEL_RADIUS;
    v2.measure = m3508_2.speed.measure * WHEEL_RADIUS;

    // 2. 舵电机安装矫正
    s1.relative.measure = s1.absolute.measure - SERVO1_OFFSET;
    s2.relative.measure = s2.absolute.measure - SERVO2_OFFSET;

    // 3. 运动学逆解
    float x1 = v1.measure * cosf(s1.relative.measure);
    float y1 = v1.measure * sinf(s1.relative.measure);
    float x2 = v2.measure * cosf(s2.relative.measure);
    float y2 = v2.measure * sinf(s2.relative.measure);
    float z1 = v1.measure * cosf(s1.relative.measure - 135 * deg);
    float z2 = v2.measure * cosf(s2.relative.measure + 45 * deg);
    vx.chassis.measure = (x1 + x2) / 2;
    vy.chassis.measure = (y1 + y2) / 2;
    vz.measure = (z1 + z2) / 2;
    vr.measure = vz.measure / CHASSIS_RADIUS;

    // 4. 转换到云台参考系
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.gimbal.measure, vy.gimbal.measure, -gimbal_yaw);
}

// 运动学逆解
void Chassis::inverseCalc() {
    // 1. 转换到底盘参考系
    // 注意这里是换参考系，而非旋转速度矢量，速度矢量相对于绝对参考系是不会发生变化的，所以旋转角度为：云台相对于底盘的角度
    std::tie(vx.chassis.ref, vy.chassis.ref) = rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw);

    // 2. 运动学正解
    constexpr float sqrt2div2 = sqrtf(2) / 2;
    float x, y;
    // 左前舵轮
    x = vx.chassis.ref - sqrt2div2 * vz.ref;
    y = vy.chassis.ref + sqrt2div2 * vz.ref;
    if (fabsf(x) > MIN_V || fabsf(y) > MIN_V) {
        s1.relative.ref = atan2f(y, x);
    }
    v1.ref = sqrtf(powf(y, 2) + powf(x, 2));
    // 右后舵轮
    x = vx.chassis.ref + sqrt2div2 * vz.ref;
    y = vy.chassis.ref - sqrt2div2 * vz.ref;
    if (fabsf(x) > MIN_V || fabsf(y) > MIN_V) {
        s2.relative.ref = atan2f(y, x);
    }
    v2.ref = sqrtf(powf(y, 2) + powf(x, 2));

    // 3. 舵电机安装矫正
    s1.absolute.ref = s1.relative.ref + SERVO1_OFFSET;
    s2.absolute.ref = s2.relative.ref + SERVO2_OFFSET;

    // 4. 就近转向
    // 左前舵轮
    Angle angle1_err = s1.absolute.ref - s1.absolute.measure;
    if (fabsf(angle1_err) > 90.0f * deg) {
        s1.relative.ref = s1.relative.ref + 180.0f * deg;
        s1.absolute.ref = s1.absolute.ref + 180.0f * deg;
        v1.ref = -v1.ref;
    }
    // 右后舵轮
    Angle angle2_err = s2.absolute.ref - s2.absolute.measure;
    if (fabsf(angle2_err) > 90.0f * deg) {
        s2.relative.ref = s2.relative.ref + 180.0f * deg;
        s2.absolute.ref = s2.absolute.ref + 180.0f * deg;
        v2.ref = -v2.ref;
    }

    // 5. 设置电机
    m6020_1.SetAngle(s1.absolute.ref);
    m6020_2.SetAngle(s2.absolute.ref);
    m3508_1.SetSpeed(v1.ref / WHEEL_RADIUS);
    m3508_2.SetSpeed(v2.ref / WHEEL_RADIUS);
}

// 估算底盘当前功率，用于调试
void Chassis::estimatePower() {
    const float power1 = m3508_1.EstimatePower();
    const float power2 = m3508_2.EstimatePower();
    power.estimate = power1 + power2;
}

// 计算电流衰减系数，需要在电机PID计算后调用
void Chassis::calcCurrentRatio() {
    // M*w + I^2*R = P
    // kt*I*w + I^2*R = P
    // kt * xI * w + (xI)^2 * R = P
    // I^2*R * x^2 + kt*I*w * x - P = 0
    // a = I^2*R
    // b = kt*I*w
    // c = -P

    float a = 0;
    a += m3508_1.current.ref * m3508_1.current.ref * M3508::R;
    a += m3508_2.current.ref * m3508_2.current.ref * M3508::R;

    float b = 0;
    b += m3508_1.kt * m3508_1.current.ref * m3508_1.speed.measure;
    b += m3508_2.kt * m3508_2.current.ref * m3508_2.speed.measure;

    float c = -power.limit;

    // 一定为一正根和一负根（x1*x2 = c/a < 0)
    power.current_ratio = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);

    power.current_ratio = std::clamp(power.current_ratio, 0.0f, 1.0f);

    m3508_1.SetCurrentRatio(power.current_ratio);
}


void Chassis::SetEnable(const bool is_enable) {
    // 设置子设备
    m6020_1.SetEnable(is_enable);
    m6020_2.SetEnable(is_enable);
    m3508_1.SetEnable(is_enable);
    m3508_2.SetEnable(is_enable);
}

void Chassis::SetGimbalYaw(const Angle<deg> gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis::SetSpeed(const Unit<m_s>& vx, const Unit<m_s>& vy, const Unit<rpm>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->vr.ref = vr;
    this->vz.ref = vr * CHASSIS_RADIUS;
}

void Chassis::SetPowerLimit(const Unit<W>& power) {
    this->power.limit = power;
}

void Chassis::Update() {
    forwardCalc(); // 运动学正解
    inverseCalc(); // 运动学逆解

    // 电机闭环控制计算
    m6020_1.Update();
    m6020_2.Update();
    m3508_1.Update();
    m3508_2.Update();

    calcCurrentRatio(); // 功率控制
}
