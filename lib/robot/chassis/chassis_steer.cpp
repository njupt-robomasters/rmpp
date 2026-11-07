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

float Steer::control(const float vx, const float vy) {
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
    motor.set_current(pid_.calculate(error));
    pid_.track(motor.current_ref);

    // 轮速投影到当前舵向，误差达到 90° 时不驱动
    return v * std::clamp(std::cosf(error), 0.0f, 1.0f);
}

void Steer::clear() {
    pid_.clear();
    has_ref_ = false;
}

const ChassisSteer::Config& ChassisSteer::config() const {
    return config_;
}

void ChassisSteer::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE);
    for (Motor* wheel : wheel_)
        wheel->set_enable(is_enable);
    for (Steer& s : steer)
        s.motor.set_enable(is_enable);
}

void ChassisSteer::set_mode(const Mode mode) {
    this->mode = mode;
}

void ChassisSteer::set_speed(const float vx, const float vy, const float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr;
}

void ChassisSteer::set_gimbal_yaw(const Angle gimbal_yaw) {
    gimbal_yaw_ = gimbal_yaw;
}

void ChassisSteer::update() {
    // 离线检测与速度正解，离线轮速度计为零
    Vector4 v;
    for (size_t i = 0; i < wheel_.size(); i++) {
        wheel_[i]->update();
        steer[i].update();
        v[i] = wheel_[i]->is_connect ? wheel_[i]->speed * config_.wheel_radius : 0.0f;
    }
    const Vector3 measure = to_gimbal(forward(v));
    vx.measure = measure.x();
    vy.measure = measure.y();
    vz.measure = measure.z();
    wr.measure = vz.measure / config_.chassis_radius;

    power.update(wheel_);

    // 旋转：用户输入优先，否则按模式分离或跟随；失能时清空跟随 PID 与舵向
    if (!is_enable) {
        follow_pid_.clear();
        for (Steer& s : steer)
            s.clear();
        wr.ref = 0;
    } else if (wr.input != 0) {
        wr.ref = wr.input;
    } else if (mode == FOLLOW_MODE) {
        wr.ref = follow_pid_.calculate(gimbal_yaw_);
    } else {
        wr.ref = 0;
    }
    vz.ref = wr.ref * config_.chassis_radius;

    control();
}

Vector3 ChassisSteer::forward(const Vector4& v) const {
    Vector3 out = Vector3::Zero();
    for (size_t i = 0; i < steer.size(); i++) {
        const float s = steer[i].measure;
        out += v[i] * Vector3(std::cosf(s), std::sinf(s), std::cosf(s - TANGENT[i]));
    }
    return out / 4.0f;
}

Vector3 ChassisSteer::forward_force(const Vector4& f) const {
    return forward(f) * 4.0f;
}

ChassisSteer::ChassisSteer(const Config& config, const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer)
    : power(config.power),
      steer{Steer(*steer[0], config.steer_pid_config), Steer(*steer[1], config.steer_pid_config), Steer(*steer[2], config.steer_pid_config), Steer(*steer[3], config.steer_pid_config)},
      config_(config), wheel_(wheel), follow_pid_(config.follow_pid_config) {}

bool ChassisSteer::can_drive(const size_t i) const {
    return wheel_[i]->is_connect && steer[i].motor.is_connect;
}

Vector4 ChassisSteer::backward(const Vector3& v) {
    Vector4 out;
    for (size_t i = 0; i < steer.size(); i++)
        out[i] = steer[i].control(v.x() + v.z() * std::cosf(TANGENT[i]), v.y() + v.z() * std::sinf(TANGENT[i]));
    return out;
}

Vector4 ChassisSteer::backward_force(const Vector3& f) const {
    // 各轮平分期望力，投影到实测舵向；自然包含就近转舵反向和转舵未到位时的衰减
    Vector4 out;
    for (size_t i = 0; i < steer.size(); i++) {
        const float fx = (f.x() + f.z() * std::cosf(TANGENT[i])) / 4.0f;
        const float fy = (f.y() + f.z() * std::sinf(TANGENT[i])) / 4.0f;
        const float s = steer[i].measure;
        out[i] = fx * std::cosf(s) + fy * std::sinf(s);
    }
    return out;
}

Vector3 ChassisSteer::to_chassis(const Vector3& gimbal) const {
    const float c = std::cos(gimbal_yaw_), s = std::sin(gimbal_yaw_);
    return {c * gimbal.x() - s * gimbal.y(), s * gimbal.x() + c * gimbal.y(), gimbal.z()};
}

Vector3 ChassisSteer::to_gimbal(const Vector3& chassis) const {
    const float c = std::cos(gimbal_yaw_), s = std::sin(gimbal_yaw_);
    return {c * chassis.x() + s * chassis.y(), -s * chassis.x() + c * chassis.y(), chassis.z()};
}

ChassisSteerSpeed::ChassisSteerSpeed(const Config& config, const Pid::Config* wheel_pid_config,
                                     const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer)
    : ChassisSteer(config, wheel, steer), wheel_pid_{Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config)} {}

void ChassisSteerSpeed::control() {
    // 失能时清空 PID，电机已失能
    if (!is_enable) {
        for (Pid& pid : wheel_pid_)
            pid.clear();
        return;
    }

    // 速度逆解（含舵向闭环）与轮速 PID，不能驱动的轮清空 PID 并输出零
    const Vector4 v = backward(to_chassis({vx.ref, vy.ref, vz.ref}));
    Vector4 current;
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (can_drive(i)) {
            current[i] = wheel_pid_[i].calculate(v[i] / config_.wheel_radius - wheel_[i]->speed);
        } else {
            wheel_pid_[i].clear();
            current[i] = 0;
        }
    }

    // 功率整体衰减，不拆旋转分量
    power.control(wheel_, current, Vector4::Zero(), false);

    // 轮速 PID 跟踪功率衰减与电机限幅后的实际电流
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (can_drive(i))
            wheel_pid_[i].track(wheel_[i]->current_ref);
    }
}

ChassisSteerTorque::ChassisSteerTorque(const Config& config, const Pid::Config* vxyz_pid_config,
                                       const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer)
    : ChassisSteer(config, wheel, steer), vx_pid_(vxyz_pid_config), vy_pid_(vxyz_pid_config), vz_pid_(vxyz_pid_config) {}

void ChassisSteerTorque::control() {
    // 力学正解，离线轮力计为零
    Vector4 f;
    for (size_t i = 0; i < wheel_.size(); i++)
        f[i] = wheel_[i]->is_connect ? wheel_[i]->torque / config_.wheel_radius : 0.0f;
    const Vector3 measure = to_gimbal(forward_force(f));
    fx.measure = measure.x();
    fy.measure = measure.y();
    fz.measure = measure.z();

    // 失能时清空 PID，电机已失能
    if (!is_enable) {
        vx_pid_.clear();
        vy_pid_.clear();
        vz_pid_.clear();
        fx.ref = fy.ref = fz.ref = 0;
        return;
    }

    // 舵向由速度逆解决定
    backward(to_chassis({vx.ref, vy.ref, vz.ref}));

    // 速度 PID 输出牵引力，平移与旋转分别力逆解为轮电流；不能驱动的轮输出零
    const Vector3 force = to_chassis({vx_pid_.calculate(vx.ref - vx.measure), vy_pid_.calculate(vy.ref - vy.measure), 0});
    const Vector4 f_xy = backward_force(force);
    const Vector4 f_z = backward_force({0, 0, vz_pid_.calculate(vz.ref - vz.measure)});
    Vector4 current_xy, current_z;
    for (size_t i = 0; i < wheel_.size(); i++) {
        const float k = can_drive(i) ? config_.wheel_radius / wheel_[i]->config.drive.kt : 0.0f;
        current_xy[i] = f_xy[i] * k;
        current_z[i] = f_z[i] * k;
    }
    power.control(wheel_, current_xy, current_z, std::fabs(wr.measure) > config_.reserve_wr);

    // 实际输出力经力学正解换回云台系，速度 PID 跟踪
    Vector4 f_out;
    for (size_t i = 0; i < wheel_.size(); i++)
        f_out[i] = wheel_[i]->torque_ref / config_.wheel_radius;
    const Vector3 out = to_gimbal(forward_force(f_out));
    fx.ref = out.x();
    fy.ref = out.y();
    fz.ref = out.z();
    vx_pid_.track(fx.ref);
    vy_pid_.track(fy.ref);
    vz_pid_.track(fz.ref);
}
