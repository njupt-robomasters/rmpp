#include "chassis_dual_steer.hpp"
#include "math/angle.hpp"
#include "math/conversions.hpp"
#include "math/math.hpp"
#include <cmath>

using math::Angle;
using math::deg;
using math::pct;

static const float SQRT2DIV2 = std::sqrt(2.0f) / 2.0f;

ChassisDualSteer::ChassisDualSteer(const Config& config, const Motors& motor) : Chassis(config), motor(motor) {}

void ChassisDualSteer::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.set_enable(is_enable);
    motor.w2.set_enable(is_enable);
    motor.s1.set_enable(is_enable);
    motor.s2.set_enable(is_enable);
}

void ChassisDualSteer::update_ui(Ui& ui) {
    ui.robot.w1 = motor.w1.is_connect ? Ui::GREEN : Ui::PINK;
    ui.robot.w4 = motor.w2.is_connect ? Ui::GREEN : Ui::PINK;
    ui.robot.s1 = motor.s1.is_connect ? Ui::GREEN : Ui::PINK;
    ui.robot.s4 = motor.s2.is_connect ? Ui::GREEN : Ui::PINK;
}

void ChassisDualSteer::on_loop() {
    Chassis::on_loop();

    // 更新电机
    motor.w1.on_loop();
    motor.w2.on_loop();
    motor.s1.on_loop();
    motor.s2.on_loop();

    power_control_after_motor_update();
}

void ChassisDualSteer::forward() {
    // 读取舵向
    s1.measure = motor.s1.angle.measure;
    s2.measure = motor.s2.angle.measure;

    // 读取电机转速
    const float v1 = motor.w1.speed.measure * config.wheel_radius;
    const float v2 = motor.w2.speed.measure * config.wheel_radius;

    // 速度正解
    float vx{}, vy{}, vz{};
    // 两舵轮速度正解 begin
    const float vx1 = v1 * std::cosf(s1.measure);
    const float vy1 = v1 * std::sinf(s1.measure);
    const float vz1 = v1 * std::cosf(s1.measure - 135 * deg);
    const float vx2 = v2 * std::cosf(s2.measure);
    const float vy2 = v2 * std::sinf(s2.measure);
    const float vz2 = v2 * std::cosf(s2.measure + 45 * deg);
    vx = (vx1 + vx2) / 2;
    vy = (vy1 + vy2) / 2;
    vz = (vz1 + vz2) / 2;
    // 两舵轮速度正解 end
    std::tie(this->vx.measure, this->vy.measure) = math::rotate(vx, vy, -gimbal_yaw_);
    this->vz.measure = vz;
    wr.measure = vz / config.chassis_radius;

    // 读取电机扭矩
    const float f1 = motor.w1.torque.measure / config.wheel_radius;
    const float f2 = motor.w2.torque.measure / config.wheel_radius;

    // 力学正解
    float fx{}, fy{}, fz{};
    // 两舵轮力学正解 begin
    const float fx1 = f1 * std::cosf(s1.measure);
    const float fy1 = f1 * std::sinf(s1.measure);
    const float fz1 = f1 * std::cosf(s1.measure - 135 * deg);
    const float fx2 = f2 * std::cosf(s2.measure);
    const float fy2 = f2 * std::sinf(s2.measure);
    const float fz2 = f2 * std::cosf(s2.measure + 45 * deg);
    fx = fx1 + fx2;
    fy = fy1 + fy2;
    fz = fz1 + fz2;
    // 两舵轮力学正解 end
    std::tie(this->fx.measure, this->fy.measure) = math::rotate(fx, fy, -gimbal_yaw_);
    this->fz.measure = fz;
}

void ChassisDualSteer::backward() {
    // 速度逆解
    auto [vx, vy] = math::rotate(this->vx.ref, this->vy.ref, gimbal_yaw_);
    auto vz = this->vz.ref;
    float v1{}, v2{};
    // 两舵轮速度逆解 begin
    // 左前舵轮
    const float vx1 = vx - SQRT2DIV2 * vz;
    const float vy1 = vy + SQRT2DIV2 * vz;
    v1 = std::sqrtf(vx1 * vx1 + vy1 * vy1);
    if (v1 > MIN_V) { // 最小转舵速度
        s1.ref_v = std::atan2f(vy1, vx1);
        if (std::fabsf(Angle(s1.ref_v - s1.measure)) > 90 * deg) { // 就近转舵
            s1.ref_v += 180 * deg;
            is_invert_v1_ = true;
        } else {
            is_invert_v1_ = false;
        }
    }
    // 右后舵轮
    const float vx2 = vx + SQRT2DIV2 * vz;
    const float vy2 = vy - SQRT2DIV2 * vz;
    v2 = std::sqrtf(vx2 * vx2 + vy2 * vy2);
    if (v2 > MIN_V) { // 最小转舵速度
        s2.ref_v = std::atan2f(vy2, vx2);
        if (std::fabsf(Angle(s2.ref_v - s2.measure)) > 90 * deg) { // 就近转舵
            s2.ref_v += 180 * deg;
            is_invert_v2_ = true;
        } else {
            is_invert_v2_ = false;
        }
    }
    // 两舵轮速度逆解 end

    // 设置电机转速
    if (is_invert_v1_)
        v1 = -v1;
    if (is_invert_v2_)
        v2 = -v2;
    motor.w1.set_speed(v1 / config.wheel_radius);
    motor.w2.set_speed(v2 / config.wheel_radius);

    // 力学逆解
    auto [fx, fy] = math::rotate(this->fx.ref, this->fy.ref, gimbal_yaw_);
    auto fz = this->fz.ref;
    float f1{}, f2{};
    // 两舵轮力学逆解 begin
    // 左前舵轮
    const float fx1 = (fx - SQRT2DIV2 * fz) / 2.0f;
    const float fy1 = (fy + SQRT2DIV2 * fz) / 2.0f;
    f1 = std::sqrtf(fx1 * fx1 + fy1 * fy1);
    if (f1 > MIN_F) { // 最小转舵力
        s1.ref_f = std::atan2f(fy1, fx1);
        if (std::fabsf(Angle(s1.ref_f - s1.measure)) > 90 * deg) { // 就近转舵
            s1.ref_f += 180 * deg;
            is_invert_f1_ = true;
        } else {
            is_invert_f1_ = false;
        }
    }
    // 右后舵轮
    const float fx2 = (fx + SQRT2DIV2 * fz) / 2.0f;
    const float fy2 = (fy - SQRT2DIV2 * fz) / 2.0f;
    f2 = std::sqrtf(fx2 * fx2 + fy2 * fy2);
    if (f2 > MIN_F) { // 最小转舵力
        s2.ref_f = std::atan2f(fy2, fx2);
        if (std::fabsf(Angle(s2.ref_f - s2.measure)) > 90 * deg) { // 就近转舵
            s2.ref_f += 180 * deg;
            is_invert_f2_ = true;
        } else {
            is_invert_f2_ = false;
        }
    }
    // 两舵轮力学逆解 end

    // 设置电机扭矩
    if (is_invert_f1_)
        f1 = -f1;
    if (is_invert_f2_)
        f2 = -f2;
    motor.w1.set_torque(f1 * config.wheel_radius);
    motor.w2.set_torque(f2 * config.wheel_radius);

    // 设置舵向
    motor.s1.set_angle(s1.ref_v);
    motor.s2.set_angle(s2.ref_v);
    // motor.s1.set_angle(s1.ref_f);
    // motor.s2.set_angle(s2.ref_f);
}

void ChassisDualSteer::power_control_after_motor_update() {
    // 功率估计
    power.estimate = motor.w1.torque.measure * motor.w1.speed.measure + motor.w2.torque.measure * motor.w2.speed.measure + 3 * math::square(motor.w1.current.measure) * motor.w1.config.r + 3 * math::square(motor.w2.current.measure) * motor.w2.config.r;

    // 裁判系统设置无限功率时，读出来是65535，禁用功率控制（这时候缓冲能量还在异常工作，必须在这里禁用）
    if (power.limit == 65535)
        return;

    // 基于功率估计的功率控制
    // I^2*R + M*w= P
    // I^2*R + kt*I*w = P
    // (xI)^2*R + kt*xI*w = P
    // I^2*R * x^2 + kt*I*w * x - P = 0
    // a = I^2*R
    // b = kt*I*w
    // c = -P
    const float a = 3 * math::square(motor.w1.current.ref) * motor.w1.config.r + 3 * math::square(motor.w2.current.ref) * motor.w2.config.r;
    const float b = motor.w1.torque.ref * motor.w1.speed.measure + motor.w2.torque.ref * motor.w2.speed.measure;
    const float c = -power.limit;
    power.gain = (-b + std::sqrtf(b * b - 4 * a * c)) / (2 * a); // 一定为一正根和一负根（x1*x2 = c/a < 0)
    power.gain = math::clamp(power.gain, 0 * pct, 100 * pct);    // 钳位

    // 基于缓冲能量的功率控制
    const auto& buffer_energy = power.buffer_energy;
    if (buffer_energy < 30) {
        power.gain *= 0;
    } else {
        power.gain *= (buffer_energy - 30) / (30);
    }

    // 衰减电机扭矩
    motor.w1.set_torque(motor.w1.torque.ref * power.gain);
    motor.w2.set_torque(motor.w2.torque.ref * power.gain);
}