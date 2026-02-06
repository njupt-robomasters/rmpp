#include "Chassis_Omni.hpp"

static const float sqrt2 = std::sqrt(2.0f);
static const float sqrt2div2 = sqrt2 / 2.0f;

Chassis_Omni::Chassis_Omni(const config_t& config, const motor_t& motor, const PID::config_t* vxyz_pid_config) : Chassis(config), motor(motor),
                                                                                                                 vx_pid(vxyz_pid_config), vy_pid(vxyz_pid_config), vz_pid(vxyz_pid_config) {
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
    handleFollow(); // 底盘跟随
    forward();      // 速度、力学正解
    handleXYZPID(); // 底盘运动PID
    powerControl(); // 功率控制
    backward();     // 速度、力学逆解

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

void Chassis_Omni::handleXYZPID() {
    fx.ref = vx_pid.Calculate(vx.ref - vx.measure);
    fy.ref = vy_pid.Calculate(vy.ref - vy.measure);
    fz.ref = vz_pid.Calculate(vz.ref - vz.measure);
}

void Chassis_Omni::powerControl() {
    // 功率估计
    power.estimate_xy = fx.measure * vx.measure + fy.measure * vy.measure;
    power.estimate_z = fz.measure * vz.measure;
    power.estimate = power.estimate_xy + power.estimate_z;

    // 裁判系统设置无限功率时，读出来是65535，禁用功率控制（这时候缓冲能量还在异常工作，必须在这里禁用）
    if (power.limit == 65535 * W) return;

    // 牵引力衰减系数
    auto& gain_x = power.gain_x;
    auto& gain_y = power.gain_y;
    auto& gain_xy = power.gain_xy;
    auto& gain_z = power.gain_z;
    gain_x = gain_y = gain_xy = gain_z = 1 * ratio; // 初始化为1，方便后面*=运算

    // 基于功率估计的功率控制
    const UnitFloat<W> power_xy = fx.ref * vx.measure + fy.ref * vy.measure;
    const UnitFloat<W> power_z = fz.ref * vz.measure;
    if (power_xy > power.limit) { // 衰减移动功率，停止旋转
        gain_xy *= power.limit / power_xy;
        gain_z *= 0;
    } else if (power_xy + power_z > power.limit) { // 只衰减旋转功率
        gain_z *= (power.limit - power_xy) / power_z;
    }

    // 基于缓冲能量的功率控制
    const auto& buffer_energy = power.buffer_energy;
    if (buffer_energy < 30 * J) { // 停止移动和旋转
        gain_xy *= 0;
        gain_z *= 0;
    } else if (buffer_energy < 45 * J) { // 衰减移动功率，停止旋转
        gain_xy *= (buffer_energy - 30 * J) / (15 * J);
        gain_z *= 0;
    } else if (buffer_energy < 60 * J) { // 只衰减旋转功率
        gain_z *= (buffer_energy - 45 * J) / (15 * J);
    }

    // 按xy牵引力比例分配
    const UnitFloat<N> f_ref_total = unit::sqrt(unit::square(fx.ref) + unit::square(fy.ref));
    gain_x = 1 * ratio - (1 * ratio - gain_xy) * unit::abs(fx.ref) / f_ref_total;
    gain_y = 1 * ratio - (1 * ratio - gain_xy) * unit::abs(fy.ref) / f_ref_total;

    // 应用牵引力衰减
    fx.ref *= gain_x;
    fy.ref *= gain_y;
    fz.ref *= gain_z;
}