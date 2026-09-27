#include "gimbal_classic.hpp"

GimbalClassic::GimbalClassic(Imu& imu, const Motors& motor)
    : Gimbal(imu), motor(motor) {}

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
    motor.yaw.set_position_zero();
}

void GimbalClassic::update_ui(Ui& ui) {
    ui.robot.yaw1 = motor.yaw.is_connected ? Ui::GREEN : Ui::PINK;
    ui.robot.yaw2 = motor.yaw.is_connected ? Ui::GREEN : Ui::PINK;
    ui.robot.pitch = motor.pitch.is_connected ? Ui::GREEN : Ui::PINK;
}

void GimbalClassic::update() {
    Gimbal::update();

    // 更新电机
    motor.yaw.update_control();
    motor.pitch.update_control();
}

void GimbalClassic::forward() {
    // 从电机读取
    yaw.ecd.measure = motor.yaw.position.measure;
    pitch.ecd.measure = motor.pitch.position.measure;

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
    yaw.ecd.ref = motor.yaw.set_position(yaw.ecd.ref, -chassis_wr_ + yaw_speed); // 同时设置前馈（前馈与底盘速度方向相反，因为云台期望静止）
    pitch.ecd.ref = motor.pitch.set_position(pitch.ecd.ref, pitch_speed);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
