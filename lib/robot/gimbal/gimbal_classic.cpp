#include "gimbal_classic.hpp"
#include <algorithm>

GimbalClassic::GimbalClassic(Imu& imu, const Config& config, const Motors& motor)
    : Gimbal(imu, config), motor(motor) {}

void GimbalClassic::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    motor.yaw.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalClassic::load_yaw_offset(Storage& storage) {
    motor.yaw.set_offset(storage.read("yaw_offset"));
}

void GimbalClassic::save_yaw_offset(Storage& storage) {
    storage.write("yaw_offset", motor.yaw.config.install.offset);
}

void GimbalClassic::set_yaw_zero() {
    if (!motor.yaw.is_connect)
        return;
    const auto& config = motor.yaw.config;
    const float dir = config.install.is_invert ? -1.0f : 1.0f;
    motor.yaw.set_offset(config.install.offset + dir * motor.yaw.position_multi * config.drive.reduction);
}

void GimbalClassic::update_ui(RobotUi& ui) {
    ui.robot.yaw1 = motor.yaw.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.yaw2 = motor.yaw.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.pitch = motor.pitch.is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

void GimbalClassic::update() {
    Gimbal::update();

    // 更新电机
    motor.yaw.update();
    motor.pitch.update();
}

void GimbalClassic::forward() {
    // 从电机读取
    yaw.ecd.measure = motor.yaw.position;
    pitch.ecd.measure = motor.pitch.position;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}

void GimbalClassic::backward() {
    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 设置电机角度
    if (config_.is_pitch_limit)
        pitch.ecd.ref = Angle(std::clamp(float(pitch.ecd.ref), config_.pitch_min, config_.pitch_max));
    yaw.ecd.ref = Angle(yaw.ecd.ref);
    pitch.ecd.ref = Angle(pitch.ecd.ref);

    const float yaw_speed_ref = yaw_position_pid_.calculate(yaw.ecd.ref - yaw.ecd.measure) - chassis_wr_ + yaw_speed;
    const float pitch_speed_ref = pitch_position_pid_.calculate(pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
    const float yaw_output = yaw_speed_pid_.calculate(yaw_speed_ref - motor.yaw.speed);
    const float pitch_output = pitch_speed_pid_.calculate(pitch_speed_ref - motor.pitch.speed);
    if (config_.yaw_output == CURRENT_OUTPUT)
        motor.yaw.set_current(yaw_output);
    else
        motor.yaw.set_torque(yaw_output);
    if (config_.pitch_output == CURRENT_OUTPUT)
        motor.pitch.set_current(pitch_output);
    else
        motor.pitch.set_torque(pitch_output);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
