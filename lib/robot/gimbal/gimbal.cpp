#include "gimbal.hpp"
#include <algorithm>

void Gimbal::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    set_motors_enable(is_enable);
}

void Gimbal::set_mode(const Mode mode) {
    mode_ = mode;
}

void Gimbal::set_speed(const float yaw_speed, const float pitch_speed) {
    yaw_speed_ = yaw_speed;
    pitch_speed_ = pitch_speed;
}

void Gimbal::set_angle(const Angle yaw, const Angle pitch) {
    if (mode_ == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode_ == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
    yaw_speed_ = pitch_speed_ = 0;
}

void Gimbal::set_chassis_wr(const float chassis_wr) {
    chassis_wr_ = chassis_wr;
}

void Gimbal::update(const Angle imu_yaw, const Angle imu_pitch) {
    const float dt = std::clamp(update_timer_.restart(), DT_MIN, DT_MAX);

    // 读取姿态与电机反馈
    yaw.imu.measure = imu_yaw;
    pitch.imu.measure = imu_pitch;
    update_motors();

    // 更新云台转动
    handle_motion(dt);

    // 角度正逆解
    forward();
    backward(dt);
}

Gimbal::Gimbal(const Config& config)
    : config_(config), yaw_speed_pid_(config.yaw_speed_pid_config), yaw_position_pid_(config.yaw_position_pid_config),
      pitch_speed_pid_(config.pitch_speed_pid_config), pitch_position_pid_(config.pitch_position_pid_config) {}

bool Gimbal::check_yaw(const bool is_connect) {
    if (is_enable_ && is_connect && is_yaw_ready_)
        return true;

    yaw.ecd.ref = yaw.ecd.measure;
    yaw.imu.ref = yaw.imu.measure;
    yaw_speed_pid_.clear();
    yaw_position_pid_.clear();
    is_yaw_ready_ = is_enable_ && is_connect;
    return is_yaw_ready_;
}

bool Gimbal::check_pitch(const bool is_connect) {
    if (is_enable_ && is_connect && is_pitch_ready_)
        return true;

    pitch.ecd.ref = pitch.ecd.measure;
    pitch.imu.ref = pitch.imu.measure;
    pitch_speed_pid_.clear();
    pitch_position_pid_.clear();
    is_pitch_ready_ = is_enable_ && is_connect;
    return is_pitch_ready_;
}

float Gimbal::set_output(Motor& motor, const PidOutput type, const float output) {
    if (type == CURRENT_OUTPUT) {
        motor.set_current(output);
        return motor.current_ref;
    }
    motor.set_torque(output);
    return motor.torque_ref;
}

void Gimbal::handle_motion(const float dt) {
    if (!is_enable_)
        return;

    const Angle yaw_delta = yaw_speed_ * dt;
    const Angle pitch_delta = pitch_speed_ * dt;
    if (mode_ == ECD_MODE) {
        yaw.ecd.ref += yaw_delta;
        pitch.ecd.ref += pitch_delta;
    } else if (mode_ == IMU_MODE) {
        yaw.imu.ref += yaw_delta;
        pitch.imu.ref += pitch_delta;
    }
}
