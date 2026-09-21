#include "gimbal_dual_yaw.hpp"

GimbalDualYaw::GimbalDualYaw(Imu& imu, const Motors& motor) : Gimbal(imu), motor(motor) {}

void GimbalDualYaw::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    motor.pitch.set_enable(is_enable);
}

void GimbalDualYaw::load_yaw_offset(FlashDb& flashdb) {
    motor.yaw1.config.offset = flashdb.read("yaw_offset");
}

void GimbalDualYaw::save_yaw_offset(FlashDb& flashdb) {
    flashdb.write("yaw_offset", motor.yaw1.config.offset);
}

void GimbalDualYaw::set_yaw_zero() {
    motor.yaw1.config.offset = motor.yaw1.raw.angle;
}

void GimbalDualYaw::update_ui(Ui& ui) {
    ui.robot.yaw1 = motor.yaw1.is_connect ? Ui::GREEN : Ui::PINK;
    ui.robot.yaw2 = motor.yaw2.is_connect ? Ui::GREEN : Ui::PINK;
    ui.robot.pitch = motor.pitch.is_connect ? Ui::GREEN : Ui::PINK;
}

void GimbalDualYaw::on_loop() {
    Gimbal::on_loop();

    // 更新电机
    motor.yaw1.on_loop();
    motor.yaw2.on_loop();
    motor.pitch.on_loop();
}

void GimbalDualYaw::forward() {
    // 从电机读取
    yaw1.measure = motor.yaw1.angle.measure;
    yaw2.measure = motor.yaw2.angle.measure;
    pitch.ecd.measure = motor.pitch.angle.measure;

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
    yaw1.ref = motor.yaw1.set_angle(yaw1.ref, -chassis_wr_ + yaw_speed);
    yaw2.ref = motor.yaw2.set_angle(yaw2.ref, -chassis_wr_ + yaw_speed - motor.yaw1.speed.measure);
    pitch.ecd.ref = motor.pitch.set_angle(pitch.ecd.ref, pitch_speed);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw1.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
