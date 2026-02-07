#include "Chassis_Omni.hpp"

static const float sqrt2 = std::sqrt(2.0f);
static const float sqrt2div2 = sqrt2 / 2.0f;

Chassis_Omni::Chassis_Omni(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {
    // 目前使用底盘速度PID，不使用轮速PID
    motor.w1.config.control_mode = Motor::OPEN_LOOP_MODE;
    motor.w2.config.control_mode = Motor::OPEN_LOOP_MODE;
    motor.w3.config.control_mode = Motor::OPEN_LOOP_MODE;
    motor.w4.config.control_mode = Motor::OPEN_LOOP_MODE;
}

void Chassis_Omni::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.SetEnable(is_enable);
    motor.w2.SetEnable(is_enable);
    motor.w3.SetEnable(is_enable);
    motor.w4.SetEnable(is_enable);
}

void Chassis_Omni::OnLoop() {
    forward();      // 速度和力学正解
    calcPID();      // 计算PID
    powerControl(); // 功率控制
    backward();     // 速度和力学逆解

    // 更新电机
    motor.w1.OnLoop();
    motor.w2.OnLoop();
    motor.w3.OnLoop();
    motor.w4.OnLoop();
}

void Chassis_Omni::forward() {
    // 读取电机转速
    const UnitFloat<m_s> v1 = motor.w1.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v2 = motor.w2.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v3 = motor.w3.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v4 = motor.w4.speed.measure * config.wheel_radius;

    // 速度正解
    UnitFloat<m_s> vx, vy, vz;
    // 全向轮速度正解 begin
    vx = sqrt2 * (-v1 - v2 + v3 + v4) / 4.0f;
    vy = sqrt2 * (+v1 - v2 - v3 + v4) / 4.0f;
    vz = (+v1 + v2 + v3 + v4) / 4.0f;
    // 全向轮速度正解 end
    std::tie(this->vx.measure, this->vy.measure) = unit::rotate(vx, vy, -gimbal_yaw);
    this->vz.measure = vz;
    wr.measure = vz / config.chassis_radius;

    // 读取电机扭矩
    const UnitFloat<N> f1 = motor.w1.torque.measure / config.wheel_radius;
    const UnitFloat<N> f2 = motor.w2.torque.measure / config.wheel_radius;
    const UnitFloat<N> f3 = motor.w3.torque.measure / config.wheel_radius;
    const UnitFloat<N> f4 = motor.w4.torque.measure / config.wheel_radius;

    // 力学正解
    UnitFloat<N> fx, fy, fz;
    // 全向轮力学正解 begin
    fx = sqrt2div2 * (-f1 - f2 + f3 + f4);
    fy = sqrt2div2 * (+f1 - f2 - f3 + f4);
    fz = f1 + f2 + f3 + f4;
    // 全向轮力学正解 end
    std::tie(this->fx.measure, this->fy.measure) = unit::rotate(fx, fy, -gimbal_yaw);
    this->fz.measure = fz;
}

void Chassis_Omni::backward() {
    // 速度逆解
    auto [vx, vy] = unit::rotate(this->vx.ref, this->vy.ref, gimbal_yaw);
    auto vz = this->vz.ref;
    UnitFloat<m_s> v1, v2, v3, v4;
    // 全向轮速度逆解 begin
    v1 = sqrt2div2 * (-vx + vy) + vz;
    v2 = sqrt2div2 * (-vx - vy) + vz;
    v3 = sqrt2div2 * (+vx - vy) + vz;
    v4 = sqrt2div2 * (+vx + vy) + vz;
    // 全向轮速度逆解 end

    // 设置电机转速
    // 目前使用底盘速度PID，不使用轮速PID，这里设置不会生效，仅用于调试
    motor.w1.SetSpeed(v1 / config.wheel_radius);
    motor.w2.SetSpeed(v2 / config.wheel_radius);
    motor.w3.SetSpeed(v3 / config.wheel_radius);
    motor.w4.SetSpeed(v4 / config.wheel_radius);

    // 力学逆解
    auto [fx, fy] = unit::rotate(this->fx.ref, this->fy.ref, gimbal_yaw);
    auto fz = this->fz.ref;
    UnitFloat<N> f1, f2, f3, f4;
    // 全向轮力学逆解 begin
    f1 = (sqrt2 * (-fx + fy) + fz) / 4.0f;
    f2 = (sqrt2 * (-fx - fy) + fz) / 4.0f;
    f3 = (sqrt2 * (+fx - fy) + fz) / 4.0f;
    f4 = (sqrt2 * (+fx + fy) + fz) / 4.0f;
    // 全向轮力学逆解 end

    // 设置电机扭矩
    motor.w1.SetTorque(f1 * config.wheel_radius);
    motor.w2.SetTorque(f2 * config.wheel_radius);
    motor.w3.SetTorque(f3 * config.wheel_radius);
    motor.w4.SetTorque(f4 * config.wheel_radius);
}