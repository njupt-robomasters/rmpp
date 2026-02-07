#include "Chassis_DualSteer.hpp"

static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis_DualSteer::Chassis_DualSteer(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {
    // 目前使用底盘速度PID，不使用轮速PID
    motor.w1.config.control_mode = Motor::OPEN_LOOP_MODE;
    motor.w2.config.control_mode = Motor::OPEN_LOOP_MODE;
}

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
    forward();      // 速度和力学正解
    calcPID();      // 计算PID
    // powerControl(); // 功率控制
    backward();     // 速度和力学逆解

    // 更新电机
    motor.w1.OnLoop();
    motor.w2.OnLoop();
    motor.s1.OnLoop();
    motor.s2.OnLoop();
}

void Chassis_DualSteer::forward() {
    // 读取舵向
    s1.measure = motor.s1.angle.measure;
    s2.measure = motor.s2.angle.measure;

    // 读取电机转速
    const UnitFloat<m_s> v1 = motor.w1.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v2 = motor.w2.speed.measure * config.wheel_radius;

    // 速度正解
    UnitFloat<m_s> vx, vy, vz;
    // 两舵轮速度正解 begin
    const UnitFloat<m_s> vx1 = v1 * unit::cos(s1.measure);
    const UnitFloat<m_s> vy1 = v1 * unit::sin(s1.measure);
    const UnitFloat<m_s> vz1 = v1 * unit::cos(s1.measure - 135 * deg);
    const UnitFloat<m_s> vx2 = v2 * unit::cos(s2.measure);
    const UnitFloat<m_s> vy2 = v2 * unit::sin(s2.measure);
    const UnitFloat<m_s> vz2 = v2 * unit::cos(s2.measure + 45 * deg);
    vx = (vx1 + vx2) / 2;
    vy = (vy1 + vy2) / 2;
    vz = (vz1 + vz2) / 2;
    // 两舵轮速度正解 end
    std::tie(this->vx.measure, this->vy.measure) = unit::rotate(vx, vy, -gimbal_yaw);
    this->vz.measure = vz;
    wr.measure = vz / config.chassis_radius;

    // 读取电机扭矩
    const UnitFloat<N> f1 = motor.w1.torque.measure / config.wheel_radius;
    const UnitFloat<N> f2 = motor.w2.torque.measure / config.wheel_radius;

    // 力学正解
    UnitFloat<N> fx, fy, fz;
    // 两舵轮力学正解 begin
    const UnitFloat<N> fx1 = f1 * unit::cos(s1.measure);
    const UnitFloat<N> fy1 = f1 * unit::sin(s1.measure);
    const UnitFloat<N> fz1 = f1 * unit::cos(s1.measure - 135 * deg);
    const UnitFloat<N> fx2 = f2 * unit::cos(s2.measure);
    const UnitFloat<N> fy2 = f2 * unit::sin(s2.measure);
    const UnitFloat<N> fz2 = f2 * unit::cos(s2.measure + 45 * deg);
    fx = fx1 + fx2;
    fy = fy1 + fy2;
    fz = fz1 + fz2;
    // 两舵轮力学正解 end
    std::tie(this->fx.measure, this->fy.measure) = unit::rotate(fx, fy, -gimbal_yaw);
    this->fz.measure = fz;
}

void Chassis_DualSteer::backward() {
    // 速度逆解
    auto [vx, vy] = unit::rotate(this->vx.ref, this->vy.ref, gimbal_yaw);
    auto vz = this->vz.ref;
    UnitFloat<m_s> v1, v2;
    // 两舵轮速度逆解 begin
    // 左前舵轮
    const UnitFloat<m_s> vx1 = vx - sqrt2div2 * vz;
    const UnitFloat<m_s> vy1 = vy + sqrt2div2 * vz;
    v1 = unit::sqrt(vx1 * vx1 + vy1 * vy1);
    // 右后舵轮
    const UnitFloat<m_s> vx2 = vx + sqrt2div2 * vz;
    const UnitFloat<m_s> vy2 = vy - sqrt2div2 * vz;
    v2 = unit::sqrt(vx2 * vx2 + vy2 * vy2);
    // 两舵轮速度逆解 end

    // 设置电机转速
    // 目前使用底盘速度PID，不使用轮速PID，这里设置不会生效，仅用于调试
    motor.w1.SetSpeed(v1 / config.wheel_radius);
    motor.w2.SetSpeed(v2 / config.wheel_radius);

    // 力学逆解
    auto [fx, fy] = unit::rotate(this->fx.ref, this->fy.ref, gimbal_yaw);
    auto fz = this->fz.ref;
    UnitFloat<N> f1, f2;
    // 两舵轮力学逆解 begin
    // 左前舵轮
    const UnitFloat<N> fx1 = (fx - sqrt2div2 * fz) / 2.0f;
    const UnitFloat<N> fy1 = (fy + sqrt2div2 * fz) / 2.0f;
    f1 = unit::sqrt(fx1 * fx1 + fy1 * fy1);
    if (f1 > MIN_F) { // 最小转舵力
        s1.ref = unit::atan2(fy1, fx1);
        if (unit::abs(Angle(s1.ref - s1.measure)) > 90 * deg) { // 就近转舵
            s1.ref += 180 * deg;
            is_invert_f1 = true;
        } else {
            is_invert_f1 = false;
        }
    }
    // 右后舵轮
    const UnitFloat<m_s> fx2 = (fx + sqrt2div2 * fz) / 2.0f;
    const UnitFloat<m_s> fy2 = (fy - sqrt2div2 * fz) / 2.0f;
    f2 = unit::sqrt(fx2 * fx2 + fy2 * fy2);
    if (f2 > MIN_F) { // 最小转舵力
        s2.ref = unit::atan2(fy2, fx2);
        if (unit::abs(Angle(s2.ref - s2.measure)) > 90 * deg) { // 就近转舵
            s2.ref += 180 * deg;
            is_invert_f2 = true;
        } else {
            is_invert_f2 = false;
        }
    }
    // 两舵轮力学逆解 end

    // 设置电机扭矩
    if (is_invert_f1) f1 = -f1;
    if (is_invert_f2) f2 = -f2;
    motor.w1.SetTorque(f1 * config.wheel_radius);
    motor.w2.SetTorque(f2 * config.wheel_radius);

    // 设置舵向
    motor.s1.SetAngle(s1.ref);
    motor.s2.SetAngle(s2.ref);
}

void Chassis_DualSteer::powerControlAfterMotorUpdate() {
    // 功率估计
    power.estimate = motor.w1.torque.measure * motor.w1.speed.measure
        + motor.w2.torque.measure * motor.w2.speed.measure
        + 3 * unit::square(motor.w1.current.measure) * motor.w1.config.R
        + 3 * unit::square(motor.w2.current.measure) * motor.w2.config.R;

    // 裁判系统设置无限功率时，读出来是65535，禁用功率控制（这时候缓冲能量还在异常工作，必须在这里禁用）
    if (power.limit == 65535 * W) return;

    // 基于功率估计的功率控制
    // I^2*R + M*w= P
    // I^2*R + kt*I*w = P
    // (xI)^2*R + kt*xI*w = P
    // I^2*R * x^2 + kt*I*w * x - P = 0
    // a = I^2*R
    // b = kt*I*w
    // c = -P
    const UnitFloat<> a = 3 * unit::square(motor.w1.current.ref) * motor.w1.config.R
        + 3 * unit::square(motor.w2.current.ref) * motor.w2.config.R;
    const UnitFloat<> b = motor.w1.torque.ref * motor.w1.speed.measure
        + motor.w2.torque.ref * motor.w2.speed.measure;
    const UnitFloat<> c = -power.limit;
    power.gain = (-b + unit::sqrt(b * b - 4 * a * c)) / (2 * a); // 一定为一正根和一负根（x1*x2 = c/a < 0)
    power.gain = unit::clamp(power.gain, 0 * pct, 100 * pct);    // 钳位

    // 基于缓冲能量的功率控制
    const auto& buffer_energy = power.buffer_energy;
    if (buffer_energy < 30 * J) {
        power.gain *= 0;
    } else if (buffer_energy < 60 * J) {
        power.gain *= (buffer_energy - 30 * J) / (30 * J);
    }

    // 衰减电机扭矩
    motor.w1.SetTorque(motor.w1.torque.ref * power.gain);
    motor.w2.SetTorque(motor.w2.torque.ref * power.gain);
}