#include "chassis_matrix.hpp"
#include <algorithm>
#include <cmath>

void ChassisMatrix::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    for (Motor* wheel : this->wheel)
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

void ChassisMatrix::set_reserve_wr(const float reserve_wr) {
    this->reserve_wr = reserve_wr;
}

void ChassisMatrix::set_gimbal_yaw(const Angle gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void ChassisMatrix::set_power_ref(const float power_ref) {
    this->power_ref = power_ref;
}

void ChassisMatrix::update() {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    // 离线检测与速度正解，离线轮反馈已清零
    Vector4f v;
    for (int i = 0; i < 4; i++) {
        wheel[i]->update();
        v[i] = wheel[i]->speed * config.wheel_radius;
    }
    const Vector3f measure = rotate(forward_speed(v), -gimbal_yaw);
    vx.measure = measure.x();
    vy.measure = measure.y();
    vz.measure = measure.z();
    wr.measure = vz.measure / config.chassis_radius;

    // 轮电机功率估计供 SWD 与超电实测对照
    power.estimate = MotorPower::estimate(wheel, config.wheel_resistance);

    // 旋转：用户输入优先，否则按模式分离或跟随；不跟随时清空跟随 PID
    if (is_enable && wr.input == 0 && mode == FOLLOW_MODE) {
        wr.ref = follow_pid.calculate(dt, gimbal_yaw);
    } else {
        follow_pid.clear();
        wr.ref = is_enable ? wr.input : 0;
    }
    vz.ref = wr.ref * config.chassis_radius;

    control(dt);
}

Vector3f ChassisMatrix::forward_speed(const Vector4f& v) const {
    return j_pinv * v;
}

Vector4f ChassisMatrix::backward_speed(const Vector3f& v) const {
    return j * v;
}

Vector3f ChassisMatrix::forward_force(const Vector4f& f) const {
    return j.transpose() * f;
}

Vector4f ChassisMatrix::backward_force(const Vector3f& f) const {
    return j_pinv.transpose() * f;
}

ChassisMatrix::ChassisMatrix(const std::array<Motor*, 4>& wheel, const Config& config)
    : config(config),
      j(Eigen::Map<const Eigen::Matrix<float, 4, 3, Eigen::RowMajor>>(config.kinematics[0])),
      j_pinv((j.transpose() * j).inverse() * j.transpose()),
      wheel{wheel[0], wheel[1], wheel[2], wheel[3]},
      // 列满秩，左伪逆 (JᵀJ)⁻¹Jᵀ
      follow_pid(config.follow_pid_config) {}

ChassisMatrixSpeed::ChassisMatrixSpeed(const std::array<Motor*, 4>& wheel, const Config& config)
    : ChassisMatrix(wheel, config),
      wheel_pid{Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config), Pid(config.speed_pid_config)} {}

void ChassisMatrixSpeed::control(const float dt) {
    // 失能时清空 PID，电机已失能
    if (!is_enable) {
        for (Pid& pid : wheel_pid)
            pid.clear();
        return;
    }

    // 速度逆解与轮速 PID，离线轮清空 PID 并输出零
    const Vector4f v = backward_speed(rotate({vx.ref, vy.ref, vz.ref}, gimbal_yaw));
    Vector4f current;
    for (int i = 0; i < 4; i++) {
        if (wheel[i]->is_connect) {
            current[i] = wheel_pid[i].calculate(dt, v[i] / config.wheel_radius - wheel[i]->speed);
        } else {
            wheel_pid[i].clear();
            current[i] = 0;
        }
    }

    // 旋转分量：电流按力正解取 z，再单独力逆解；互相对抗的内力归平移
    Vector4f current_z = backward_force({0, 0, forward_force(current).z()});
    for (int i = 0; i < 4; i++) {
        if (!wheel[i]->is_connect)
            current_z[i] = 0;
    }
    power.gain = MotorPower::limit_current(wheel, config.wheel_resistance, current - current_z, current_z, std::fabs(wr.measure) > reserve_wr, power_ref);

    // 轮速 PID 跟踪功率衰减与电机限幅后的实际电流
    for (int i = 0; i < 4; i++) {
        if (wheel[i]->is_connect)
            wheel_pid[i].track(wheel[i]->current_ref);
    }
}

ChassisMatrixTorque::ChassisMatrixTorque(const std::array<Motor*, 4>& wheel, const Config& config)
    : ChassisMatrix(wheel, config),
      vx_pid(config.speed_pid_config),
      vy_pid(config.speed_pid_config),
      vz_pid(config.speed_pid_config) {}

void ChassisMatrixTorque::control(const float dt) {
    // 力学正解，离线轮反馈已清零
    Vector4f f;
    for (int i = 0; i < 4; i++)
        f[i] = wheel[i]->torque / config.wheel_radius;
    const Vector3f measure = rotate(forward_force(f), -gimbal_yaw);
    fx.measure = measure.x();
    fy.measure = measure.y();
    fz.measure = measure.z();

    // 失能时清空 PID，电机已失能
    if (!is_enable) {
        vx_pid.clear();
        vy_pid.clear();
        vz_pid.clear();
        fx.ref = fy.ref = fz.ref = 0;
        return;
    }

    // 速度 PID 输出牵引力，平移与旋转分别力逆解为轮电流；离线轮输出零
    const Vector3f force = rotate({vx_pid.calculate(dt, vx.ref - vx.measure), vy_pid.calculate(dt, vy.ref - vy.measure), 0}, gimbal_yaw);
    const Vector4f f_xy = backward_force(force);
    const Vector4f f_z = backward_force({0, 0, vz_pid.calculate(dt, vz.ref - vz.measure)});
    Vector4f current_xy, current_z;
    for (int i = 0; i < 4; i++) {
        const auto& drive = *wheel[i];
        const float k = wheel[i]->is_connect ? config.wheel_radius * drive.max_current / drive.max_torque : 0.0f;
        current_xy[i] = f_xy[i] * k;
        current_z[i] = f_z[i] * k;
    }
    power.gain = MotorPower::limit_current(wheel, config.wheel_resistance, current_xy, current_z, std::fabs(wr.measure) > reserve_wr, power_ref);

    // 实际输出力经力学正解换回云台系，速度 PID 跟踪
    Vector4f f_out;
    for (int i = 0; i < 4; i++)
        f_out[i] = wheel[i]->torque_ref / config.wheel_radius;
    const Vector3f out = rotate(forward_force(f_out), -gimbal_yaw);
    fx.ref = out.x();
    fy.ref = out.y();
    fz.ref = out.z();
    vx_pid.track(fx.ref);
    vy_pid.track(fy.ref);
    vz_pid.track(fz.ref);
}
