#include "chassis_steer.hpp"
#include <algorithm>
#include <cmath>

// 正方形布置：x 向前、y 向左；各轮旋转切向方向（左前、左后、右后、右前）
// 轮速度 = 平移速度 + vz 沿切向，即 (vx + vz·cos t, vy + vz·sin t)
static constexpr std::array<float, 4> TANGENT = {135 * deg, -135 * deg, -45 * deg, 45 * deg};

Steer::Steer(Motor& motor, const Pid::Config* pid_config)
    : motor(motor), pid_(pid_config) {}

void Steer::update() {
    motor.update();
    measure = motor.position;
}

float Steer::control(const float dt, const float vx, const float vy) {
    // 离线时清空，重连后从当前舵向开始，避免突然转舵
    if (!motor.is_connect) {
        clear();
        motor.set_current(0);
        return 0;
    }
    if (!has_ref_) {
        ref = measure;
        has_ref_ = true;
    }

    // 低于最小转舵速度时保持舵向，避免零速抖动；误差超过 90° 时就近转舵并反转轮子
    float v = std::hypotf(vx, vy);
    if (v > MIN_V) {
        ref = std::atan2f(vy, vx);
        if (std::fabsf(Angle(ref - measure)) > 90 * deg) {
            ref += 180 * deg;
            v = -v;
        }
    } else {
        v = 0;
    }

    // 舵向闭环，误差取最短角度；PID 跟踪电机限幅后的实际电流
    const Angle error = ref - measure;
    motor.set_current(pid_.calculate(dt, error));
    pid_.track(motor.current_ref);

    // 轮速投影到当前舵向，误差达到 90° 时不驱动
    return v * std::clamp(std::cosf(error), 0.0f, 1.0f);
}

void Steer::clear() {
    pid_.clear();
    has_ref_ = false;
}

void ChassisSteer::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    for (Motor* wheel : wheel_)
        wheel->set_enable(is_enable);
    for (Steer& s : steer)
        s.motor.set_enable(is_enable);
}

void ChassisSteer::set_mode(const Mode mode) {
    mode_ = mode;
}

void ChassisSteer::set_speed(const float vx, const float vy, const float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr;
}

void ChassisSteer::set_reserve_wr(const float reserve_wr) {
    reserve_wr_ = reserve_wr;
}

void ChassisSteer::set_gimbal_yaw(const Angle gimbal_yaw) {
    gimbal_yaw_ = gimbal_yaw;
}

void ChassisSteer::set_power_ref(const float power_ref) {
    power_ref_ = power_ref;
}

void ChassisSteer::update() {
    const float dt = std::clamp(update_timer_.restart(), DT_MIN, DT_MAX);

    // 离线检测与速度正解，离线轮反馈已清零
    Vector4f v;
    for (int i = 0; i < 4; i++) {
        wheel_[i]->update();
        steer[i].update();
        v[i] = wheel_[i]->speed * config_.wheel_radius;
    }
    const Vector3f measure = rotate(forward_speed(v), -gimbal_yaw_);
    vx.measure = measure.x();
    vy.measure = measure.y();
    vz.measure = measure.z();
    wr.measure = vz.measure / config_.chassis_radius;

    // 轮电机功率估计供 SWD 与超电实测对照
    power.estimate = MotorPower::estimate(wheel_, config_.wheel_resistance);

    // 失能时清空舵向
    if (!is_enable_) {
        for (Steer& s : steer)
            s.clear();
    }

    // 旋转：用户输入优先，否则按模式分离或跟随；不跟随时清空跟随 PID
    if (is_enable_ && wr.input == 0 && mode_ == FOLLOW_MODE) {
        wr.ref = follow_pid_.calculate(dt, gimbal_yaw_);
    } else {
        follow_pid_.clear();
        wr.ref = is_enable_ ? wr.input : 0;
    }
    vz.ref = wr.ref * config_.chassis_radius;

    control(dt);
}

Vector3f ChassisSteer::forward_speed(const Vector4f& v) const {
    Vector3f out = Vector3f::Zero();
    for (int i = 0; i < 4; i++) {
        const float s = steer[i].measure;
        out += v[i] * Vector3f(std::cosf(s), std::sinf(s), std::cosf(s - TANGENT[i]));
    }
    return out / 4.0f;
}

Vector3f ChassisSteer::forward_force(const Vector4f& f) const {
    return forward_speed(f) * 4.0f;
}

ChassisSteer::ChassisSteer(const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config)
    : steer{Steer(*steer[0], config.steer_pid_config), Steer(*steer[1], config.steer_pid_config), Steer(*steer[2], config.steer_pid_config), Steer(*steer[3], config.steer_pid_config)},
      config_(config), wheel_{wheel[0], wheel[1], wheel[2], wheel[3]}, follow_pid_(config.follow_pid_config) {}

bool ChassisSteer::can_drive(const size_t i) const {
    return wheel_[i]->is_connect && steer[i].motor.is_connect;
}

Vector4f ChassisSteer::backward_speed(const float dt, const Vector3f& v) {
    Vector4f out;
    for (int i = 0; i < 4; i++)
        out[i] = steer[i].control(dt, v.x() + v.z() * std::cosf(TANGENT[i]), v.y() + v.z() * std::sinf(TANGENT[i]));
    return out;
}

Vector4f ChassisSteer::backward_force(const Vector3f& f) const {
    // 各轮平分期望力，投影到实测舵向；自然包含就近转舵反向和转舵未到位时的衰减
    Vector4f out;
    for (int i = 0; i < 4; i++) {
        const float fx = (f.x() + f.z() * std::cosf(TANGENT[i])) / 4.0f;
        const float fy = (f.y() + f.z() * std::sinf(TANGENT[i])) / 4.0f;
        const float s = steer[i].measure;
        out[i] = fx * std::cosf(s) + fy * std::sinf(s);
    }
    return out;
}

ChassisSteerSpeed::ChassisSteerSpeed(const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config)
    : ChassisSteer(wheel, steer, config),
      wheel_pid_{Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config)} {}

void ChassisSteerSpeed::control(const float dt) {
    // 失能时清空 PID，电机已失能
    if (!is_enable_) {
        for (Pid& pid : wheel_pid_)
            pid.clear();
        return;
    }

    // 速度逆解（含舵向闭环）与轮速 PID，不能驱动的轮清空 PID 并输出零
    const Vector4f v = backward_speed(dt, rotate({vx.ref, vy.ref, vz.ref}, gimbal_yaw_));
    Vector4f current;
    for (int i = 0; i < 4; i++) {
        if (can_drive(i)) {
            current[i] = wheel_pid_[i].calculate(dt, v[i] / config_.wheel_radius - wheel_[i]->speed);
        } else {
            wheel_pid_[i].clear();
            current[i] = 0;
        }
    }

    // 功率整体衰减，不拆旋转分量
    power.gain = MotorPower::limit_current(wheel_, config_.wheel_resistance, current, Vector4f::Zero(), false, power_ref_);

    // 轮速 PID 跟踪功率衰减与电机限幅后的实际电流
    for (int i = 0; i < 4; i++) {
        if (can_drive(i))
            wheel_pid_[i].track(wheel_[i]->current_ref);
    }
}

ChassisSteerTorque::ChassisSteerTorque(const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer, const Config& config)
    : ChassisSteer(wheel, steer, config),
      vx_pid_(config.speed_pid_config), vy_pid_(config.speed_pid_config), vz_pid_(config.speed_pid_config) {}

void ChassisSteerTorque::control(const float dt) {
    // 力学正解，离线轮反馈已清零
    Vector4f f;
    for (int i = 0; i < 4; i++)
        f[i] = wheel_[i]->torque / config_.wheel_radius;
    const Vector3f measure = rotate(forward_force(f), -gimbal_yaw_);
    fx.measure = measure.x();
    fy.measure = measure.y();
    fz.measure = measure.z();

    // 失能时清空 PID，电机已失能
    if (!is_enable_) {
        vx_pid_.clear();
        vy_pid_.clear();
        vz_pid_.clear();
        fx.ref = fy.ref = fz.ref = 0;
        return;
    }

    // 舵向由速度逆解决定
    backward_speed(dt, rotate({vx.ref, vy.ref, vz.ref}, gimbal_yaw_));

    // 速度 PID 输出牵引力，平移与旋转分别力逆解为轮电流；不能驱动的轮输出零
    const Vector3f force = rotate({vx_pid_.calculate(dt, vx.ref - vx.measure), vy_pid_.calculate(dt, vy.ref - vy.measure), 0}, gimbal_yaw_);
    const Vector4f f_xy = backward_force(force);
    const Vector4f f_z = backward_force({0, 0, vz_pid_.calculate(dt, vz.ref - vz.measure)});
    Vector4f current_xy, current_z;
    for (int i = 0; i < 4; i++) {
        const auto& drive = wheel_[i]->config.drive;
        const float k = can_drive(i) ? config_.wheel_radius * drive.max_current / drive.max_torque : 0.0f;
        current_xy[i] = f_xy[i] * k;
        current_z[i] = f_z[i] * k;
    }
    power.gain = MotorPower::limit_current(wheel_, config_.wheel_resistance, current_xy, current_z, std::fabs(wr.measure) > reserve_wr_, power_ref_);

    // 实际输出力经力学正解换回云台系，速度 PID 跟踪
    Vector4f f_out;
    for (int i = 0; i < 4; i++)
        f_out[i] = wheel_[i]->torque_ref / config_.wheel_radius;
    const Vector3f out = rotate(forward_force(f_out), -gimbal_yaw_);
    fx.ref = out.x();
    fy.ref = out.y();
    fz.ref = out.z();
    vx_pid_.track(fx.ref);
    vy_pid_.track(fy.ref);
    vz_pid_.track(fz.ref);
}
