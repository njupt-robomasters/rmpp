#include "gimbal_classic.hpp"

GimbalClassic::GimbalClassic(const Motors& motor, const Config& config)
    : Gimbal(config),
      motor(motor) {}

void GimbalClassic::set_motors_enable(const bool is_enable) {
    motor.yaw.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalClassic::update_motors() {
    motor.yaw.update();
    motor.pitch.update();
}

void GimbalClassic::forward() {
    // 从电机读取
    yaw.ecd.measure = motor.yaw.position;
    pitch.ecd.measure = motor.pitch.position;
}

void GimbalClassic::backward(const float dt) {
    // 失能或离线的轴目标跟随实测，yaw、pitch 互不影响
    const bool can_yaw = check_yaw(motor.yaw.is_connect);
    const bool can_pitch = check_pitch(motor.pitch.is_connect);

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
        set_output(motor.yaw, config.yaw_output, yaw_speed_pid.calculate(dt, speed_ref - motor.yaw.speed));
    } else {
        motor.yaw.set_current(0);
    }

    // pitch 串级 PID
    if (can_pitch) {
        const float speed_ref = pitch_position_pid.calculate(dt, pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
        set_output(motor.pitch, config.pitch_output, pitch_speed_pid.calculate(dt, speed_ref - motor.pitch.speed));
    } else {
        motor.pitch.set_current(0);
    }
}
