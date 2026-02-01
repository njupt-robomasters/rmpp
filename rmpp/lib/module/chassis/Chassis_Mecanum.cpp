#include "Chassis_Mecanum.hpp"

static const float sqrt2 = std::sqrt(2.0f);
static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis_Mecanum::Chassis_Mecanum(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {}

void Chassis_Mecanum::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.SetEnable(is_enable);
    motor.w2.SetEnable(is_enable);
    motor.w3.SetEnable(is_enable);
    motor.w4.SetEnable(is_enable);
}

void Chassis_Mecanum::OnLoop() {
    // 底盘跟随
    calcFollow();

    // 运动学解算
    backward();
    forward();

    // 更新电机
    motor.w1.OnLoop();
    motor.w2.OnLoop();
    motor.w3.OnLoop();
    motor.w4.OnLoop();
}

void Chassis_Mecanum::forward() {
    // 1. 转换到底盘参考系
    // 注意这里是换参考系，而非旋转速度矢量，所以旋转角度为：底盘 -> 云台的角度
    std::tie(vx.chassis.ref, vy.chassis.ref) = unit::rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw);

    // 2. 运动学正解
    vz.ref = wr.ref * config.chassis_radius;
    v1.ref = -vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;
    v2.ref = -vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
    v3.ref = +vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
    v4.ref = +vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;

    // 3. 设置电机转速
    motor.w1.SetSpeed(v1.ref / config.wheel_radius);
    motor.w2.SetSpeed(v2.ref / config.wheel_radius);
    motor.w3.SetSpeed(v3.ref / config.wheel_radius);
    motor.w4.SetSpeed(v4.ref / config.wheel_radius);
}

void Chassis_Mecanum::backward() {
    // 1.读取电机转速
    v1.measure = motor.w1.speed.measure * config.wheel_radius;
    v2.measure = motor.w2.speed.measure * config.wheel_radius;
    v3.measure = motor.w3.speed.measure * config.wheel_radius;
    v4.measure = motor.w4.speed.measure * config.wheel_radius;

    // 2. 运动学逆解
    vx.chassis.measure = (-v1.measure - v2.measure + v3.measure + v4.measure) / 4.0f;
    vy.chassis.measure = (+v1.measure - v2.measure - v3.measure + v4.measure) / 4.0f;
    vz.measure = sqrt2div2 * (+v1.measure + v2.measure + v3.measure + v4.measure) / 4.0f;
    wr.measure = vz.measure / config.chassis_radius;

    // 3. 转换到云台参考系
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw);
}