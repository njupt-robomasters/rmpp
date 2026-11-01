#include "gimbal.hpp"
#include "math/angle.hpp"

Gimbal::Gimbal(Attitude& attitude, const Config& config)
    : attitude(attitude), config_(config), yaw_speed_pid_(config.yaw_speed_pid_config), yaw_position_pid_(config.yaw_position_pid_config),
      pitch_speed_pid_(config.pitch_speed_pid_config), pitch_position_pid_(config.pitch_position_pid_config) {}

void Gimbal::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    if (!is_enable) {
        yaw_speed_pid_.clear();
        yaw_position_pid_.clear();
        pitch_speed_pid_.clear();
        pitch_position_pid_.clear();
    }

    // 这里设置电机
}

void Gimbal::set_mode(const Mode mode) {
    this->mode = mode;
}

void Gimbal::set_angle(Angle yaw, Angle pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
    yaw_speed = pitch_speed = 0.0f;
}

void Gimbal::set_speed(float yaw_speed, float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void Gimbal::set_chassis_wr(float chassis_wr) {
    this->chassis_wr_ = chassis_wr;
}

void Gimbal::update() {
    attitude.update();

    // 更新云台转动
    handle_motion();

    // 角度正逆解
    forward();
    backward();

    // 这里更新电机
}

void Gimbal::handle_motion() {
    const float dt = timer_motion_.restart();
    if (is_enable) {
        const Angle yaw_delta = yaw_speed * dt;
        const Angle pitch_delta = pitch_speed * dt;
        if (mode == ECD_MODE) {
            yaw.ecd.ref += yaw_delta;
            pitch.ecd.ref += pitch_delta;
        } else if (mode == IMU_MODE) {
            yaw.imu.ref += yaw_delta;
            pitch.imu.ref += pitch_delta;
        }
    }
}
