#include "gimbal_dual_yaw.hpp"
#include <algorithm>

GimbalDualYaw::GimbalDualYaw(const Motors& motor, const Config& config)
    : config(config),
      motor(motor),
      yaw_speed_pid(config.yaw_speed_pid_config),
      yaw_angle_pid(config.yaw_angle_pid_config),
      pitch_speed_pid(config.pitch_speed_pid_config),
      pitch_angle_pid(config.pitch_angle_pid_config) {}

void GimbalDualYaw::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalDualYaw::set_mode(const Mode mode) {
    this->mode = mode;
}

void GimbalDualYaw::set_speed(const float yaw_speed, const float pitch_speed) {
    this->yaw_speed = yaw_speed;
    this->pitch_speed = pitch_speed;
}

void GimbalDualYaw::set_angle(const Angle yaw, const Angle pitch) {
    if (mode == ECD_MODE) {
        this->yaw.ecd.ref = yaw;
        this->pitch.ecd.ref = pitch;
    } else if (mode == IMU_MODE) {
        this->yaw.imu.ref = yaw;
        this->pitch.imu.ref = pitch;
    }
    yaw_speed = pitch_speed = 0;
}

void GimbalDualYaw::set_chassis_wr(const float chassis_wr) {
    this->chassis_wr = chassis_wr;
}

void GimbalDualYaw::update(const Angle imu_yaw, const Angle imu_pitch) {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    // 读取姿态与电机反馈
    yaw.imu.measure = imu_yaw;
    pitch.imu.measure = imu_pitch;
    motor.yaw1.update();
    motor.yaw2.update();
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

    // 从电机读取，离线电机保持最后一次反馈
    yaw1.measure = motor.yaw1.angle;
    yaw2.measure = motor.yaw2.angle;
    pitch.ecd.measure = motor.pitch.angle;

    // 合并大小yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 失能或离线时目标跟随实测并清空 PID，恢复首帧从实测开始
    const bool can_yaw = is_enable && (motor.yaw1.is_connect || motor.yaw2.is_connect);
    const bool can_pitch = is_enable && motor.pitch.is_connect;
    if (!can_yaw || !is_yaw_ready) {
        yaw.ecd.ref = yaw.ecd.measure;
        yaw.imu.ref = yaw.imu.measure;
        yaw_speed_pid.clear();
        yaw_angle_pid.clear();
    }
    if (!can_pitch || !is_pitch_ready) {
        pitch.ecd.ref = pitch.ecd.measure;
        pitch.imu.ref = pitch.imu.measure;
        pitch_speed_pid.clear();
        pitch_angle_pid.clear();
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

    // 分配大小yaw
    yaw1.ref = yaw.ecd.ref;                // 大yaw直接设置为目标值
    yaw2.ref = yaw.ecd.ref - yaw1.measure; // 大yaw未转到目标，小yaw来补偿

    // 软件限位
    pitch.ecd.ref = Angle(std::clamp(static_cast<float>(pitch.ecd.ref), config.pitch_min, config.pitch_max));
    // 传递软件限位到imu参考系
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;

    // yaw 串级 PID，速度反馈取在线的大 yaw，否则取小 yaw；离线的一侧输出零，在线的一侧单独保持指向
    if (can_yaw) {
        const float speed = motor.yaw1.is_connect ? motor.yaw1.speed : motor.yaw2.speed;
        const float speed_ref = yaw_angle_pid.calculate(dt, yaw.ecd.ref - yaw.ecd.measure) - chassis_wr + yaw_speed;
        const float output = yaw_speed_pid.calculate(dt, speed_ref - speed);
        motor.yaw1.set_output(motor.yaw1.is_connect ? output : 0.0f);
        motor.yaw2.set_output(motor.yaw2.is_connect ? output : 0.0f);
    } else {
        motor.yaw1.set_output(0);
        motor.yaw2.set_output(0);
    }

    // pitch 串级 PID
    if (can_pitch) {
        const float speed_ref = pitch_angle_pid.calculate(dt, pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
        motor.pitch.set_output(pitch_speed_pid.calculate(dt, speed_ref - motor.pitch.speed));
    } else {
        motor.pitch.set_output(0);
    }

    // 按大 yaw 目标回写 IMU 参考系
    yaw.imu.ref = yaw1.ref + yaw.imu_minus_ecd;
}
