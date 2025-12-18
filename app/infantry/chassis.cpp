#include "chassis.hpp"

static const float sqrt2 = std::sqrt(2.0f);
static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis::Chassis(PID::param_t* wheel_pid, PID::param_t* follow_pid) :
    m_wheel1(1, 1),
    m_wheel2(1, 2),
    m_wheel3(1, 3),
    m_wheel4(1, 4) {
    // 设置电机PID参数
    m_wheel1.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid);
    m_wheel2.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid);
    m_wheel3.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid);
    m_wheel4.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, wheel_pid);

    // 设置底盘跟随PID参数
    this->follow_pid.SetParam(follow_pid);

    // 设置电机减速比
    m_wheel1.SetReduction(14.0f);
    m_wheel2.SetReduction(14.0f);
    m_wheel3.SetReduction(14.0f);
    m_wheel4.SetReduction(14.0f);

    // 设置电流力矩系数
    constexpr UnitFloat Kt = 0.525f * Nm_A;
    m_wheel1.SetKt(Kt);
    m_wheel2.SetKt(Kt);
    m_wheel3.SetKt(Kt);
    m_wheel4.SetKt(Kt);

    // 设置电机正方向
    m_wheel1.SetInvert(false);
    m_wheel2.SetInvert(false);
    m_wheel3.SetInvert(false);
    m_wheel4.SetInvert(false);
}

void Chassis::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    m_wheel1.SetEnable(is_enable);
    m_wheel2.SetEnable(is_enable);
    m_wheel3.SetEnable(is_enable);
    m_wheel4.SetEnable(is_enable);
}

void Chassis::OnLoop() {
    // 底盘跟随
    calcFollow();

    // 运动学解算
    speedBackward();
    speedForward();

    // 电机PID计算
    m_wheel1.OnLoop();
    m_wheel2.OnLoop();
    m_wheel3.OnLoop();
    m_wheel4.OnLoop();

    // 功率控制
    powerControl();
}

void Chassis::speedForward() {
    // 1. 转换到底盘参考系
    // 注意这里是换参考系，而非旋转速度矢量，所以旋转角度为：底盘 -> 云台的角度
    std::tie(vx.chassis.ref, vy.chassis.ref) = unit::rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw);
    vz.ref = wr.ref.sum * CHASSIS_RADIUS;

    // 2. 运动学正解
    v1.ref = -sqrt2div2 * vx.chassis.ref + sqrt2div2 * vy.chassis.ref + vz.ref;
    v2.ref = -sqrt2div2 * vx.chassis.ref - sqrt2div2 * vy.chassis.ref + vz.ref;
    v3.ref = +sqrt2div2 * vx.chassis.ref - sqrt2div2 * vy.chassis.ref + vz.ref;
    v4.ref = +sqrt2div2 * vx.chassis.ref + sqrt2div2 * vy.chassis.ref + vz.ref;

    // 3. 设置电机转速
    m_wheel1.SetSpeed(v1.ref / WHEEL_RADIUS);
    m_wheel2.SetSpeed(v2.ref / WHEEL_RADIUS);
    m_wheel3.SetSpeed(v3.ref / WHEEL_RADIUS);
    m_wheel4.SetSpeed(v4.ref / WHEEL_RADIUS);
}

void Chassis::speedBackward() {
    // 1.读取电机转速
    v1.measure = m_wheel1.speed.measure * WHEEL_RADIUS;
    v2.measure = m_wheel2.speed.measure * WHEEL_RADIUS;
    v3.measure = m_wheel3.speed.measure * WHEEL_RADIUS;
    v4.measure = m_wheel4.speed.measure * WHEEL_RADIUS;

    // 2. 运动学逆解
    vx.chassis.measure = sqrt2 * (-v1.measure - v2.measure + v3.measure + v4.measure) / 4.0f;
    vy.chassis.measure = sqrt2 * (+v1.measure - v2.measure - v3.measure + v4.measure) / 4.0f;
    vz.measure = (v1.measure + v2.measure + v3.measure + v4.measure) / 4.0f;
    wr.measure = vz.measure / CHASSIS_RADIUS;

    // 3. 转换到云台参考系
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw);
}

void Chassis::powerControl() {
    // M*w + I^2*R = P
    // kt*I*w + I^2*R = P
    // kt * xI * w + (xI)^2 * R = P
    // I^2*R * x^2 + kt*I*w * x - P = 0
    // a = I^2*R
    // b = kt*I*w
    // c = -P

    UnitFloat a;
    a += 3 * unit::square(m_wheel1.current.ref) * m_wheel1.R;
    a += 3 * unit::square(m_wheel2.current.ref) * m_wheel2.R;
    a += 3 * unit::square(m_wheel3.current.ref) * m_wheel3.R;
    a += 3 * unit::square(m_wheel4.current.ref) * m_wheel4.R;

    UnitFloat b;
    b += m_wheel1.Kt * m_wheel1.current.ref * m_wheel1.speed.measure;
    b += m_wheel2.Kt * m_wheel2.current.ref * m_wheel2.speed.measure;
    b += m_wheel3.Kt * m_wheel3.current.ref * m_wheel3.speed.measure;
    b += m_wheel4.Kt * m_wheel4.current.ref * m_wheel4.speed.measure;

    const UnitFloat c = -power.limit;

    // 一定为一正根和一负根（x1*x2 = c/a < 0)
    power.current_ratio = (-b + unit::sqrt(b * b - 4 * a * c)) / (2 * a);

    // 钳位
    power.current_ratio = unit::clamp(power.current_ratio, 0.0f * default_unit, 1.0f * default_unit);

    // 设置电机电流衰减
    m_wheel1.SetCurrentRatio(power.current_ratio);
    m_wheel2.SetCurrentRatio(power.current_ratio);
    m_wheel3.SetCurrentRatio(power.current_ratio);
    m_wheel4.SetCurrentRatio(power.current_ratio);

    // 估算底盘当前功率
    power.estimate = m_wheel1.power.total + m_wheel2.power.total + m_wheel3.power.total + m_wheel4.power.total;
}