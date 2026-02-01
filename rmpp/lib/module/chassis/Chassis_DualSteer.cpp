#include "Chassis_DualSteer.hpp"

static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis_DualSteer::Chassis_DualSteer(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {}

void Chassis_DualSteer::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return; // 防止重复设置
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.SetEnable(is_enable);
    motor.w2.SetEnable(is_enable);
    motor.s1.SetEnable(is_enable);
    motor.s2.SetEnable(is_enable);
}

void Chassis_DualSteer::OnLoop() {
    // 底盘跟随
    calcFollow();

    // 运动学解算
    backward();
    forward();

    // 更新电机
    motor.w1.OnLoop();
    motor.w2.OnLoop();
    motor.s1.OnLoop();
    motor.s2.OnLoop();
}

void Chassis_DualSteer::forward() {
    // 1. 转换到底盘参考系
    // 注意这里是换参考系，而非旋转速度矢量，所以旋转角度为：底盘 -> 云台的角度
    std::tie(vx.chassis.ref, vy.chassis.ref) = unit::rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw);

    // 2. 运动学正解
    vz.ref = wr.ref * config.chassis_radius;
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
    if (unit::abs(Angle(s1.ref - s1.measure)) > 90 * deg) {
        s1.ref = s1.ref + 180 * deg;
        v1.ref = -v1.ref;
    }
    // 右后舵轮
    if (unit::abs(Angle(s2.ref - s2.measure)) > 90 * deg) {
        s2.ref = s2.ref + 180 * deg;
        v2.ref = -v2.ref;
    }

    // 4. 设置电机
    motor.w1.SetSpeed(v1.ref / config.wheel_radius);
    motor.w2.SetSpeed(v2.ref / config.wheel_radius);
    motor.s1.SetAngle(s1.ref);
    motor.s2.SetAngle(s2.ref);
}

void Chassis_DualSteer::backward() {
    // 1. 读取轮电机
    v1.measure = motor.w1.speed.measure * config.wheel_radius;
    v2.measure = motor.w2.speed.measure * config.wheel_radius;
    s1.measure = motor.s1.angle.measure;
    s2.measure = motor.s2.angle.measure;

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
    wr.measure = vz.measure / config.chassis_radius;

    // 3. 转换到云台参考系
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw);
}