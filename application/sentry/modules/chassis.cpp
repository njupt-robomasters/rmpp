#include "chassis.hpp"
#include "utils.hpp"

Chassis::Chassis(PID::param_t* servo_pid_param, PID::param_t* wheel_pid_param) :
    m_servo1(1, 0x207),
    m_servo2(1, 0x208),
    m_wheel1(1, 0x201),
    m_wheel2(1, 0x202) {
    // 设置PID参数
    m_servo1.SetPIDParam(servo_pid_param);
    m_servo2.SetPIDParam(servo_pid_param);
    m_wheel1.SetPIDParam(wheel_pid_param);
    m_wheel2.SetPIDParam(wheel_pid_param);

    // 设置减速比
    m_wheel1.SetReduction(14.0f);
    m_wheel2.SetReduction(14.0f);

    // 反转电机
    m_servo1.SetInvert(true);
    m_servo2.SetInvert(true);
}

void Chassis::forwardCalc() {
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
    m_servo1.SetAngle(s1.absolute.ref);
    m_servo2.SetAngle(s2.absolute.ref);
    m_wheel1.SetSpeed(v1.ref / WHEEL_RADIUS);
    m_wheel2.SetSpeed(v2.ref / WHEEL_RADIUS);
}

void Chassis::backwardCalc() {
    // 1. 读取电机
    s1.absolute.measure = m_servo1.angle.measure;
    s2.absolute.measure = m_servo2.angle.measure;
    v1.measure = m_wheel1.speed.measure * WHEEL_RADIUS;
    v2.measure = m_wheel2.speed.measure * WHEEL_RADIUS;

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

// 估算底盘当前功率，用于调试
void Chassis::estimatePower() {
    const float power1 = m_wheel1.EstimatePower();
    const float power2 = m_wheel2.EstimatePower();
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
    a += m_wheel1.current.ref * m_wheel1.current.ref * M3508::R;
    a += m_wheel2.current.ref * m_wheel2.current.ref * M3508::R;

    float b = 0;
    b += m_wheel1.kt * m_wheel1.current.ref * m_wheel1.speed.measure;
    b += m_wheel2.kt * m_wheel2.current.ref * m_wheel2.speed.measure;

    float c = -power.limit;

    // 一定为一正根和一负根（x1*x2 = c/a < 0)
    power.current_ratio = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);

    // 钳位
    power.current_ratio = clamp(power.current_ratio, 0.0f, 1.0f);

    m_wheel1.SetCurrentRatio(power.current_ratio);
    m_wheel2.SetCurrentRatio(power.current_ratio);
}


void Chassis::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
    m_servo1.SetEnable(is_enable);
    m_servo2.SetEnable(is_enable);
    m_wheel1.SetEnable(is_enable);
    m_wheel2.SetEnable(is_enable);
}

void Chassis::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->vr.ref = vr;
    this->vz.ref = vr * CHASSIS_RADIUS;
}

void Chassis::SetPowerLimit(const UnitFloat<>& power) {
    this->power.limit = power;
}

void Chassis::Update() {
    backwardCalc();
    forwardCalc();

    m_servo1.Update();
    m_servo2.Update();
    m_wheel1.Update();
    m_wheel2.Update();

    calcCurrentRatio(); // 功率控制
}
