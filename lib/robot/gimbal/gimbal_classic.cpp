#include "gimbal_classic.hpp"
#include <algorithm>

GimbalClassic::GimbalClassic(const Motors& motor, const Config& config)
    : config(config),
      motor(motor),
      yaw_speed_pid(config.yaw_speed_pid_config),
      yaw_position_pid(config.yaw_position_pid_config),
      pitch_speed_pid(config.pitch_speed_pid_config),
      pitch_position_pid(config.pitch_position_pid_config) {}

void GimbalClassic::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.yaw.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalClassic::set_mode(const Mode mode) {
    this->mode = mode;
}

void GimbalClassic::set_speed(const float yaw_speed, const float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void GimbalClassic::set_angle(const Angle yaw, const Angle pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
    yaw_speed = pitch_speed = 0;
}

void GimbalClassic::set_chassis_wr(const float chassis_wr) {
    this->chassis_wr = chassis_wr;
}

void GimbalClassic::update(const Angle imu_yaw, const Angle imu_pitch) {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    // 读取姿态与电机反馈
    yaw.imu.measure = imu_yaw;
    pitch.imu.measure = imu_pitch;
    motor.yaw.update();
    motor.pitch.update();

    // 按目标速度积分角度
    if (is_enable) {
        if (mode == ECD_MODE) {
            yaw.ecd.ref += yaw_speed * dt;
            pitch.ecd.ref += pitch_speed * dt;
        } else if (mode == IMU_MODE) {
            yaw.imu.ref += yaw_speed * dt;
            pitch.imu.ref += pitch_speed * dt;
        }
    }

    // 从电机读取
    yaw.ecd.measure = motor.yaw.position;
    pitch.ecd.measure = motor.pitch.position;

    // 失能或离线时目标跟随实测并清空 PID，恢复首帧从实测开始
    const bool can_yaw = is_enable && motor.yaw.is_connect;
    const bool can_pitch = is_enable && motor.pitch.is_connect;
    if (!can_yaw || !is_yaw_ready) {
        yaw.ecd.ref = yaw.ecd.measure;
        yaw.imu.ref = yaw.imu.measure;
        yaw_speed_pid.clear();
        yaw_position_pid.clear();
    }
    if (!can_pitch || !is_pitch_ready) {
        pitch.ecd.ref = pitch.ecd.measure;
        pitch.imu.ref = pitch.imu.measure;
        pitch_speed_pid.clear();
        pitch_position_pid.clear();
    }
    is_yaw_ready = can_yaw;
    is_pitch_ready = can_pitch;

    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 软件限位
    pitch.ecd.ref = Angle(std::clamp(static_cast<float>(pitch.ecd.ref), config.pitch_min, config.pitch_max));
    // 传递软件限位到imu参考系
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;

    // yaw 串级 PID
    if (can_yaw) {
        const float speed_ref = yaw_position_pid.calculate(dt, yaw.ecd.ref - yaw.ecd.measure) - chassis_wr + yaw_speed;
        motor.yaw.set_output(yaw_speed_pid.calculate(dt, speed_ref - motor.yaw.speed));
    } else {
        motor.yaw.set_output(0);
    }

    // pitch 串级 PID
    if (can_pitch) {
        const float speed_ref = pitch_position_pid.calculate(dt, pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
        motor.pitch.set_output(pitch_speed_pid.calculate(dt, speed_ref - motor.pitch.speed));
    } else {
        motor.pitch.set_output(0);
    }
}
