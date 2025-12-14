#include "chassis.hpp"

const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis::Chassis(PID::param_t* servo_pid_param, PID::param_t* wheel_pid_param) :
    m_servo1(1, 3),
    m_servo2(1, 4),
    m1(1, 1),
    m_wheel2(1, 2) {
    // 设置电机PID参数
    m_servo1.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, servo_pid_param);
    m_servo2.SetPID(Motor::ANGLE_MODE, Motor::CURRENT_TYPE, servo_pid_param);
    m1.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid_param);
    m_wheel2.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid_param);

    // 设置电机减速比
    m1.SetReduction(14.0f);
    m_wheel2.SetReduction(14.0f);

    // 设置电机正方向
    m_servo1.SetInvert(true);
    m_servo2.SetInvert(true);

    // 设置电机偏移
    m_servo1.SetOffset(SERVO1_OFFSET);
    m_servo2.SetOffset(SERVO2_OFFSET);
}

void Chassis::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return; // 防止重复设置
    this->is_enable = is_enable;
    m_servo1.SetEnable(is_enable);
    m_servo2.SetEnable(is_enable);
    m1.SetEnable(is_enable);
    m_wheel2.SetEnable(is_enable);
}

void Chassis::OnLoop() {
    // 运动学解算
    backwardCalc();
    forwardCalc();

    // 电机PID计算
    m_servo1.OnLoop();
    m_servo2.OnLoop();
    m1.OnLoop();
    m_wheel2.OnLoop();

    // 功率控制
    powerControl();
}

void Chassis::forwardCalc() {
    // 1. 转换到底盘参考系
    // 注意这里是换参考系，而非旋转速度矢量，所以旋转角度为：底盘 -> 云台的角度
    std::tie(vx.chassis.ref, vy.chassis.ref) = unit::rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw);

    // 2. 运动学正解
    vz.ref = vr.ref * CHASSIS_RADIUS;
    // 左前舵轮
    UnitFloat x = vx.chassis.ref - sqrt2div2 * vz.ref;
    UnitFloat y = vy.chassis.ref + sqrt2div2 * vz.ref;
    if (unit::abs(x) > MIN_V || unit::abs(y) > MIN_V) {
        s1.ref = unit::atan2(y, x);
    }
    v1.ref = unit::sqrt(unit::square(y) + unit::square(x));
    // 右后舵轮
    x = vx.chassis.ref + sqrt2div2 * vz.ref;
    y = vy.chassis.ref - sqrt2div2 * vz.ref;
    if (unit::abs(x) > MIN_V || unit::abs(y) > MIN_V) {
        s2.ref = unit::atan2(y, x);
    }
    v2.ref = unit::sqrt(unit::square(y) + unit::square(x));

    // 3. 就近转向
    // 左前舵轮
    if (unit::abs(Angle(s1.ref - s1.measure)) > 90.0f * deg) {
        s1.ref = s1.ref + 180.0f * deg;
        v1.ref = -v1.ref;
    }
    // 右后舵轮
    if (unit::abs(Angle(s2.ref - s2.measure)) > 90.0f * deg) {
        s2.ref = s2.ref + 180.0f * deg;
        v2.ref = -v2.ref;
    }

    // 4. 设置电机转速
    m_servo1.SetAngle(s1.ref);
    m_servo2.SetAngle(s2.ref);
    m1.SetSpeed(v1.ref / WHEEL_RADIUS);
    m_wheel2.SetSpeed(v2.ref / WHEEL_RADIUS);
}

void Chassis::backwardCalc() {
    // 1. 读取电机转速
    s1.measure = m_servo1.speed.measure;
    s2.measure = m_servo2.speed.measure;

    // 2. 运动学逆解
    const UnitFloat x1 = v1.measure * unit::cos(s1.measure);
    const UnitFloat y1 = v1.measure * unit::sin(s1.measure);
    const UnitFloat x2 = v2.measure * unit::cos(s2.measure);
    const UnitFloat y2 = v2.measure * unit::sin(s2.measure);
    const UnitFloat z1 = v1.measure * unit::cos(s1.measure - 135 * deg);
    const UnitFloat z2 = v2.measure * unit::cos(s2.measure + 45 * deg);
    vx.chassis.measure = (x1 + x2) / 2;
    vy.chassis.measure = (y1 + y2) / 2;
    vz.measure = (z1 + z2) / 2;
    vr.measure = vz.measure / CHASSIS_RADIUS;

    // 3. 转换到云台参考系
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.gimbal.measure, vy.gimbal.measure, -gimbal_yaw);
}

// 计算电流衰减系数，需要在电机PID计算后调用
void Chassis::powerControl() {
    // M*w + I^2*R = P
    // kt*I*w + I^2*R = P
    // kt * xI * w + (xI)^2 * R = P
    // I^2*R * x^2 + kt*I*w * x - P = 0
    // a = I^2*R
    // b = kt*I*w
    // c = -P

    UnitFloat a;
    a += 3 * unit::square(m1.current.ref) * M3508::R;
    a += 3 * unit::square(m_wheel2.current.ref) * M3508::R;

    UnitFloat b;
    b += M3508::Kt * m1.current.ref * m1.speed.measure;
    b += M3508::Kt * m_wheel2.current.ref * m_wheel2.speed.measure;

    const UnitFloat c = -power_limit;

    // 一定为一正根和一负根（x1*x2 = c/a < 0)
    current_ratio = (-b + unit::sqrt(b * b - 4 * a * c)) / (2 * a);

    // 钳位
    current_ratio = unit::clamp(current_ratio, 0.0f * default_unit, 1.0f * default_unit);

    // 设置电机电流衰减
    m1.SetCurrentRatio(current_ratio);
    m_wheel2.SetCurrentRatio(current_ratio);

    // 估算底盘当前功率，用于调试
    const UnitFloat power1 = 3 * unit::square(m1.current.ref) * M3508::R + M3508::Kt * m1.current.ref * m1.speed.measure;
    const UnitFloat power2 = 3 * unit::square(m_wheel2.current.ref) * M3508::R + M3508::Kt * m_wheel2.current.ref * m_wheel2.speed.measure;
    power_estimate = power1 + power2;
}