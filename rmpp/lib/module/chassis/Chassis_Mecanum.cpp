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
    // 速度解算
    std::tie(vx.chassis.ref, vy.chassis.ref) = unit::rotate(vx.gimbal.ref, vy.gimbal.ref, gimbal_yaw); // 转换到底盘参考系
    vz.ref = wr.ref * config.chassis_radius;
    v1.ref = -vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;
    v2.ref = -vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
    v3.ref = +vx.chassis.ref - vy.chassis.ref + sqrt2 * vz.ref;
    v4.ref = +vx.chassis.ref + vy.chassis.ref + sqrt2 * vz.ref;

    // 设置电机转速
    // motor.w1.SetSpeed(v1.ref / config.wheel_radius);
    // motor.w2.SetSpeed(v2.ref / config.wheel_radius);
    // motor.w3.SetSpeed(v3.ref / config.wheel_radius);
    // motor.w4.SetSpeed(v4.ref / config.wheel_radius);

    // 功率控制
    // 1.先行衰减xy速度，防止PID饱和后偏向
    const UnitFloat<W> power_xy_pre = fx.gimbal.measure * vx.gimbal.ref + fy.gimbal.measure * vy.gimbal.ref;
    UnitFloat<m_s> vx_ref, vy_ref;
    if (power_xy_pre > power_limit) {
        vx_ref = vx.gimbal.ref * power_limit / power_xy_pre;
        vy_ref = vy.gimbal.ref * power_limit / power_xy_pre;
    } else {
        vx_ref = vx.gimbal.ref;
        vy_ref = vy.gimbal.ref;
    }

    // 计算PID
    fx.gimbal.ref = vx_pid.Calculate(vx_ref - vx.gimbal.measure);
    fy.gimbal.ref = vy_pid.Calculate(vy_ref - vy.gimbal.measure);
    fz.ref = vz_pid.Calculate(vz.ref - vz.measure);

    // 功率控制
    // 2.基于功率估计的功率控制
    const UnitFloat<W> power_xy = fx.gimbal.ref * vx.gimbal.measure + fy.gimbal.ref * vy.gimbal.measure;
    const UnitFloat<W> power_z = fz.ref * vz.measure;
    if (power_xy > power_limit) { // 停止旋转，衰减平移功率
        fx.gimbal.ref *= power_limit / power_xy;
        fy.gimbal.ref *= power_limit / power_xy;
        fz.ref = 0 * default_unit;

        // 清空PID，防止i累计
        vx_pid.Clear();
        vy_pid.Clear();
        vz_pid.Clear();
    } else if (power_xy + power_z > power_limit) { // 只衰减旋转功率
        fz.ref *= (power_limit - power_xy) / power_z;

        // 清空PID，防止i累计
        vz_pid.Clear();
    }
    // 3.基于缓冲能量的功率控制
    if (buffer_energy < 30 * J) { // 停止平移和旋转
        fx.gimbal.ref = 0 * default_unit;
        fy.gimbal.ref = 0 * default_unit;
        fz.ref = 0 * default_unit;

        // 清空PID，防止i累计
        vx_pid.Clear();
        vy_pid.Clear();
        vz_pid.Clear();
    } else if (buffer_energy < 45 * J) { // 停止旋转，衰减平移功率
        fx.gimbal.ref *= (buffer_energy - 30 * J) / (15 * J);
        fy.gimbal.ref *= (buffer_energy - 30 * J) / (15 * J);
        fz.ref = 0 * default_unit;

        // 清空PID，防止i累计
        vx_pid.Clear();
        vy_pid.Clear();
        vz_pid.Clear();
    } else if (buffer_energy < 60 * J) { // 只衰减旋转功率
        fz.ref *= (buffer_energy - 45 * J) / (15 * J);

        // 清空PID，防止i累计
        vz_pid.Clear();
    }

    // 力学解算
    std::tie(fx.chassis.ref, fy.chassis.ref) = unit::rotate(fx.gimbal.ref, fy.gimbal.ref, gimbal_yaw); // 转换到底盘参考系
    mr.ref = fz.ref * config.chassis_radius;
    f1.ref = (-fx.chassis.ref + fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
    f2.ref = (-fx.chassis.ref - fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
    f3.ref = (+fx.chassis.ref - fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;
    f4.ref = (+fx.chassis.ref + fy.chassis.ref + sqrt2div2 * fz.ref) / 4.0f;

    // 设置电机扭矩
    motor.w1.SetTorque(f1.ref * config.wheel_radius);
    motor.w2.SetTorque(f2.ref * config.wheel_radius);
    motor.w3.SetTorque(f3.ref * config.wheel_radius);
    motor.w4.SetTorque(f4.ref * config.wheel_radius);
}

void Chassis_Mecanum::backward() {
    // 读取电机转速
    v1.measure = motor.w1.speed.measure * config.wheel_radius;
    v2.measure = motor.w2.speed.measure * config.wheel_radius;
    v3.measure = motor.w3.speed.measure * config.wheel_radius;
    v4.measure = motor.w4.speed.measure * config.wheel_radius;

    // 速度解算
    vx.chassis.measure = (-v1.measure - v2.measure + v3.measure + v4.measure) / 4.0f;
    vy.chassis.measure = (+v1.measure - v2.measure - v3.measure + v4.measure) / 4.0f;
    vz.measure = sqrt2div2 * (+v1.measure + v2.measure + v3.measure + v4.measure) / 4.0f;
    wr.measure = vz.measure / config.chassis_radius;
    std::tie(vx.gimbal.measure, vy.gimbal.measure) = unit::rotate(vx.chassis.measure, vy.chassis.measure, -gimbal_yaw); // 转换到云台参考系

    // 读取电机扭矩
    f1.measure = motor.w1.torque.measure / config.wheel_radius;
    f2.measure = motor.w2.torque.measure / config.wheel_radius;
    f3.measure = motor.w3.torque.measure / config.wheel_radius;
    f4.measure = motor.w4.torque.measure / config.wheel_radius;

    // 力学解算
    fx.chassis.measure = -f1.measure - f2.measure + f3.measure + f4.measure;
    fy.chassis.measure = +f1.measure - f2.measure - f3.measure + f4.measure;
    fz.measure = sqrt2 * (f1.measure + f2.measure + f3.measure + f4.measure);
    mr.measure = fz.measure * config.chassis_radius;
    std::tie(fx.gimbal.measure, fy.gimbal.measure) = unit::rotate(fx.chassis.measure, fy.chassis.measure, -gimbal_yaw); // 转换到云台参考系
}