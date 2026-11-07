#include "chassis_matrix.hpp"
#include <cmath>

const ChassisMatrix::Config& ChassisMatrix::config() const {
    return config_;
}

void ChassisMatrix::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE);
    for (Motor* wheel : wheel_)
        wheel->set_enable(is_enable);
}

void ChassisMatrix::set_mode(const Mode mode) {
    this->mode = mode;
}

void ChassisMatrix::set_speed(const float vx, const float vy, const float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr;
}

void ChassisMatrix::set_gimbal_yaw(const Angle gimbal_yaw) {
    gimbal_yaw_ = gimbal_yaw;
}

void ChassisMatrix::update() {
    // 离线检测与速度正解，离线轮速度计为零
    Vector4 v;
    for (size_t i = 0; i < wheel_.size(); i++) {
        wheel_[i]->update();
        v[i] = wheel_[i]->is_connect ? wheel_[i]->speed * config_.wheel_radius : 0.0f;
    }
    const Vector3 measure = to_gimbal(forward(v));
    vx.measure = measure.x();
    vy.measure = measure.y();
    vz.measure = measure.z();
    wr.measure = vz.measure / config_.chassis_radius;

    power.update(wheel_);

    // 旋转：用户输入优先，否则按模式分离或跟随；失能时清空跟随 PID
    if (!is_enable) {
        follow_pid_.clear();
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

Vector3 ChassisMatrix::forward(const Vector4& v) const {
    return j_pinv_ * v;
}

Vector4 ChassisMatrix::backward(const Vector3& v) const {
    return j_ * v;
}

Vector3 ChassisMatrix::forward_force(const Vector4& f) const {
    return j_.transpose() * f;
}

Vector4 ChassisMatrix::backward_force(const Vector3& f) const {
    return j_pinv_.transpose() * f;
}

ChassisMatrix::ChassisMatrix(const Config& config, const WheelMatrix& kinematics, const std::array<Motor*, 4>& wheel)
    : power(config.power), config_(config), wheel_(wheel), follow_pid_(config.follow_pid_config) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            j_(i, j) = kinematics[i][j];

    // 列满秩，左伪逆 (JᵀJ)⁻¹Jᵀ
    j_pinv_ = (j_.transpose() * j_).inverse() * j_.transpose();
}

Vector3 ChassisMatrix::to_chassis(const Vector3& gimbal) const {
    const float c = std::cos(gimbal_yaw_), s = std::sin(gimbal_yaw_);
    return {c * gimbal.x() - s * gimbal.y(), s * gimbal.x() + c * gimbal.y(), gimbal.z()};
}

Vector3 ChassisMatrix::to_gimbal(const Vector3& chassis) const {
    const float c = std::cos(gimbal_yaw_), s = std::sin(gimbal_yaw_);
    return {c * chassis.x() + s * chassis.y(), -s * chassis.x() + c * chassis.y(), chassis.z()};
}

ChassisMatrixSpeed::ChassisMatrixSpeed(const Config& config, const WheelMatrix& kinematics, const Pid::Config* wheel_pid_config, const std::array<Motor*, 4>& wheel)
    : ChassisMatrix(config, kinematics, wheel), wheel_pid_{Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config)} {}

void ChassisMatrixSpeed::control() {
    // 失能时清空 PID，电机已失能
    if (!is_enable) {
        for (Pid& pid : wheel_pid_)
            pid.clear();
        return;
    }

    // 速度逆解与轮速 PID，离线轮清空 PID 并输出零
    const Vector4 v = backward(to_chassis({vx.ref, vy.ref, vz.ref}));
    Vector4 current;
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (wheel_[i]->is_connect) {
            current[i] = wheel_pid_[i].calculate(v[i] / config_.wheel_radius - wheel_[i]->speed);
        } else {
            wheel_pid_[i].clear();
            current[i] = 0;
        }
    }

    // 旋转分量：电流按力正解取 z，再单独力逆解；互相对抗的内力归平移
    Vector4 current_z = backward_force({0, 0, forward_force(current).z()});
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (!wheel_[i]->is_connect)
            current_z[i] = 0;
    }
    power.control(wheel_, current - current_z, current_z, std::fabs(wr.measure) > config_.reserve_wr);

    // 轮速 PID 跟踪功率衰减与电机限幅后的实际电流
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (wheel_[i]->is_connect)
            wheel_pid_[i].track(wheel_[i]->current_ref);
    }
}

ChassisMatrixTorque::ChassisMatrixTorque(const Config& config, const WheelMatrix& kinematics, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel)
    : ChassisMatrix(config, kinematics, wheel), vx_pid_(vxyz_pid_config), vy_pid_(vxyz_pid_config), vz_pid_(vxyz_pid_config) {}

void ChassisMatrixTorque::control() {
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

    // 速度 PID 输出牵引力，平移与旋转分别力逆解为轮电流；离线轮输出零
    const Vector3 force = to_chassis({vx_pid_.calculate(vx.ref - vx.measure), vy_pid_.calculate(vy.ref - vy.measure), 0});
    const Vector4 f_xy = backward_force(force);
    const Vector4 f_z = backward_force({0, 0, vz_pid_.calculate(vz.ref - vz.measure)});
    Vector4 current_xy, current_z;
    for (size_t i = 0; i < wheel_.size(); i++) {
        const float k = wheel_[i]->is_connect ? config_.wheel_radius / wheel_[i]->config.drive.kt : 0.0f;
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
