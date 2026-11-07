#include "chassis_matrix.hpp"
#include <cmath>

void ChassisMatrix::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    for (Motor* wheel : wheel_)
        wheel->set_enable(is_enable);
}

void ChassisMatrix::set_mode(const Mode mode) {
    mode_ = mode;
}

void ChassisMatrix::set_speed(const float vx, const float vy, const float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr;
}

void ChassisMatrix::set_reserve_wr(const float reserve_wr) {
    reserve_wr_ = reserve_wr;
}

void ChassisMatrix::set_gimbal_yaw(const Angle gimbal_yaw) {
    gimbal_yaw_ = gimbal_yaw;
}

void ChassisMatrix::update() {
    // 离线检测与速度正解，离线轮反馈已清零
    Vector4f v;
    for (size_t i = 0; i < wheel_.size(); i++) {
        wheel_[i]->update();
        v[i] = wheel_[i]->speed * config_.wheel_radius;
    }
    const Vector3f measure = rotate(forward_speed(v), -gimbal_yaw_);
    vx.measure = measure.x();
    vy.measure = measure.y();
    vz.measure = measure.z();
    wr.measure = vz.measure / config_.chassis_radius;

    power_.update(wheel_);

    // 旋转：用户输入优先，否则按模式分离或跟随；失能时清空跟随 PID
    if (!is_enable_) {
        follow_pid_.clear();
        wr.ref = 0;
    } else if (wr.input != 0) {
        wr.ref = wr.input;
    } else if (mode_ == FOLLOW_MODE) {
        wr.ref = follow_pid_.calculate(gimbal_yaw_);
    } else {
        wr.ref = 0;
    }
    vz.ref = wr.ref * config_.chassis_radius;

    control();
}

Vector3f ChassisMatrix::forward_speed(const Vector4f& v) const {
    return j_pinv_ * v;
}

Vector4f ChassisMatrix::backward_speed(const Vector3f& v) const {
    return j_ * v;
}

Vector3f ChassisMatrix::forward_force(const Vector4f& f) const {
    return j_.transpose() * f;
}

Vector4f ChassisMatrix::backward_force(const Vector3f& f) const {
    return j_pinv_.transpose() * f;
}

ChassisMatrix::ChassisMatrix(const Config& config, PowerLimiter& power, const std::array<Motor*, 4>& wheel)
    : config_(config), wheel_(wheel), power_(power), follow_pid_(config.follow_pid_config) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            j_(i, j) = config.kinematics[i][j];

    // 列满秩，左伪逆 (JᵀJ)⁻¹Jᵀ
    j_pinv_ = (j_.transpose() * j_).inverse() * j_.transpose();
}

ChassisMatrixSpeed::ChassisMatrixSpeed(const Config& config, PowerLimiter& power, const std::array<Motor*, 4>& wheel)
    : ChassisMatrix(config, power, wheel),
      wheel_pid_{Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config)} {}

void ChassisMatrixSpeed::control() {
    // 失能时清空 PID，电机已失能
    if (!is_enable_) {
        for (Pid& pid : wheel_pid_)
            pid.clear();
        return;
    }

    // 速度逆解与轮速 PID，离线轮清空 PID 并输出零
    const Vector4f v = backward_speed(rotate({vx.ref, vy.ref, vz.ref}, gimbal_yaw_));
    Vector4f current;
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (wheel_[i]->is_connect) {
            current[i] = wheel_pid_[i].calculate(v[i] / config_.wheel_radius - wheel_[i]->speed);
        } else {
            wheel_pid_[i].clear();
            current[i] = 0;
        }
    }

    // 旋转分量：电流按力正解取 z，再单独力逆解；互相对抗的内力归平移
    Vector4f current_z = backward_force({0, 0, forward_force(current).z()});
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (!wheel_[i]->is_connect)
            current_z[i] = 0;
    }
    power_.control(wheel_, current - current_z, current_z, std::fabs(wr.measure) > reserve_wr_);

    // 轮速 PID 跟踪功率衰减与电机限幅后的实际电流
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (wheel_[i]->is_connect)
            wheel_pid_[i].track(wheel_[i]->current_ref);
    }
}

ChassisMatrixTorque::ChassisMatrixTorque(const Config& config, PowerLimiter& power, const std::array<Motor*, 4>& wheel)
    : ChassisMatrix(config, power, wheel),
      vx_pid_(config.speed_pid_config), vy_pid_(config.speed_pid_config), vz_pid_(config.speed_pid_config) {}

void ChassisMatrixTorque::control() {
    // 力学正解，离线轮反馈已清零
    Vector4f f;
    for (size_t i = 0; i < wheel_.size(); i++)
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

    // 速度 PID 输出牵引力，平移与旋转分别力逆解为轮电流；离线轮输出零
    const Vector3f force = rotate({vx_pid_.calculate(vx.ref - vx.measure), vy_pid_.calculate(vy.ref - vy.measure), 0}, gimbal_yaw_);
    const Vector4f f_xy = backward_force(force);
    const Vector4f f_z = backward_force({0, 0, vz_pid_.calculate(vz.ref - vz.measure)});
    Vector4f current_xy, current_z;
    for (size_t i = 0; i < wheel_.size(); i++) {
        const float k = wheel_[i]->is_connect ? config_.wheel_radius / wheel_[i]->config.drive.kt : 0.0f;
        current_xy[i] = f_xy[i] * k;
        current_z[i] = f_z[i] * k;
    }
    power_.control(wheel_, current_xy, current_z, std::fabs(wr.measure) > reserve_wr_);

    // 实际输出力经力学正解换回云台系，速度 PID 跟踪
    Vector4f f_out;
    for (size_t i = 0; i < wheel_.size(); i++)
        f_out[i] = wheel_[i]->torque_ref / config_.wheel_radius;
    const Vector3f out = rotate(forward_force(f_out), -gimbal_yaw_);
    fx.ref = out.x();
    fy.ref = out.y();
    fz.ref = out.z();
    vx_pid_.track(fx.ref);
    vy_pid_.track(fy.ref);
    vz_pid_.track(fz.ref);
}
