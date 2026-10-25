#include "gimbal_dual_yaw.hpp"
#include <algorithm>

GimbalDualYaw::GimbalDualYaw(Imu& imu, const Config& config, const Motors& motor)
    : Gimbal(imu, config), motor(motor) {}

void GimbalDualYaw::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalDualYaw::load_yaw_offset(Storage& storage) {
    motor.yaw1.set_offset(storage.read("yaw_offset"));
}

void GimbalDualYaw::save_yaw_offset(Storage& storage) {
    storage.write("yaw_offset", motor.yaw1.config.install.offset);
}

void GimbalDualYaw::set_yaw_zero() {
    if (!motor.yaw1.is_connect)
        return;
    const auto& config = motor.yaw1.config;
    const float dir = config.install.is_invert ? -1.0f : 1.0f;
    motor.yaw1.set_offset(config.install.offset + dir * motor.yaw1.position_multi * config.drive.reduction);
}

void GimbalDualYaw::update_ui(RobotUi& ui) {
    ui.robot.yaw1 = motor.yaw1.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.yaw2 = motor.yaw2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.pitch = motor.pitch.is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

void GimbalDualYaw::update() {
    Gimbal::update();

    // 更新电机
    motor.yaw1.update();
    motor.yaw2.update();
    motor.pitch.update();
}

void GimbalDualYaw::forward() {
    // 从电机读取
    yaw1.measure = motor.yaw1.position;
    yaw2.measure = motor.yaw2.position;
    pitch.ecd.measure = motor.pitch.position;

    // 合并大小yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}

void GimbalDualYaw::backward() {
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

    // 分配大小yaw
    yaw1.ref = yaw.ecd.ref;                // 大yaw直接设置为目标值
    yaw2.ref = yaw.ecd.ref - yaw1.measure; // 大yaw未转到目标，小yaw来补偿

    // 设置电机角度
    if (config_.is_pitch_limit)
        pitch.ecd.ref = Angle(std::clamp(float(pitch.ecd.ref), config_.pitch_min, config_.pitch_max));
    yaw1.ref = Angle(yaw1.ref);
    yaw2.ref = Angle(yaw2.ref);
    pitch.ecd.ref = Angle(pitch.ecd.ref);

    const float yaw_speed_ref = yaw_position_pid_.calculate(yaw.ecd.ref - yaw.ecd.measure) - chassis_wr_ + yaw_speed;
    const float pitch_speed_ref = pitch_position_pid_.calculate(pitch.ecd.ref - pitch.ecd.measure) + pitch_speed;
    const float yaw_output = yaw_speed_pid_.calculate(yaw_speed_ref - motor.yaw1.speed);
    const float pitch_output = pitch_speed_pid_.calculate(pitch_speed_ref - motor.pitch.speed);
    if (config_.yaw_output == CURRENT_OUTPUT) {
        motor.yaw1.set_current(yaw_output);
        motor.yaw2.set_current(yaw_output);
    } else {
        motor.yaw1.set_torque(yaw_output);
        motor.yaw2.set_torque(yaw_output);
    }
    if (config_.pitch_output == CURRENT_OUTPUT)
        motor.pitch.set_current(pitch_output);
    else
        motor.pitch.set_torque(pitch_output);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw1.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
