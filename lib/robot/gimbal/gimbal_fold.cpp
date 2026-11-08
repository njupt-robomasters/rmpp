#include "gimbal_fold.hpp"
#include <algorithm>
#include <cmath>

GimbalFold::GimbalFold(Attitude& attitude, const Motors& motor, const Config& config)
    : Gimbal(attitude, config.gimbal), motor(motor), fold_config_(config),
      yaw2_speed_pid_(config.yaw2.speed_pid_config), yaw2_position_pid_(config.yaw2.position_pid_config),
      pitch1_speed_pid_(config.pitch1.speed_pid_config), pitch1_position_pid_(config.pitch1.position_pid_config) {}

void GimbalFold::set_pitch1_angle(const Angle angle) {
    pitch1.ref = angle;
}

void GimbalFold::load_yaw_offset(KvStore& kv_store) {
    motor.yaw1.set_offset(kv_store.read("yaw_offset"));
}

void GimbalFold::save_yaw_offset(KvStore& kv_store) {
    kv_store.write("yaw_offset", motor.yaw1.config.install.offset);
}

void GimbalFold::set_yaw_zero() {
    if (!motor.yaw1.is_connect)
        return;
    const auto& config = motor.yaw1.config;
    const float dir = config.install.is_invert ? -1.0f : 1.0f;
    motor.yaw1.set_offset(config.install.offset + dir * motor.yaw1.position_multi * config.drive.reduction);
}

void GimbalFold::set_motors_enable(const bool is_enable) {
    is_motors_enable_ = is_enable;
    motor.yaw1.set_enable(is_enable);
    motor.yaw2.set_enable(is_enable);
    motor.pitch1.set_enable(is_enable);
    motor.pitch2.set_enable(is_enable);
}

void GimbalFold::update_motors() {
    motor.yaw1.update();
    motor.yaw2.update();
    motor.pitch1.update();
    motor.pitch2.update();
}

void GimbalFold::forward() {
    // 从电机读取，离线电机保持最后一次反馈
    yaw1.measure = motor.yaw1.position;
    yaw2.measure = motor.yaw2.position;
    pitch1.measure = motor.pitch1.position;
    pitch.ecd.measure = motor.pitch2.position;

    // 合并大小 yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 读取 imu
    yaw.imu.measure = attitude.yaw;
    pitch.imu.measure = attitude.pitch;
}

void GimbalFold::backward() {
    // 云台轴：大小 yaw 任一在线即可控制 yaw 总目标，俯仰轴独立；失能或离线时目标跟随实测
    check_yaw(motor.yaw1.is_connect || motor.yaw2.is_connect);
    const bool can_pitch2 = check_pitch(motor.pitch2.is_connect);

    // 各电机单独判断，离线电机输出零，不影响其他电机
    const bool can_yaw1 = check_motor(motor.yaw1, is_yaw1_ready_, yaw_speed_pid_, yaw_position_pid_);
    const bool can_yaw2 = check_motor(motor.yaw2, is_yaw2_ready_, yaw2_speed_pid_, yaw2_position_pid_);
    const bool can_pitch1 = check_motor(motor.pitch1, is_pitch1_ready_, pitch1_speed_pid_, pitch1_position_pid_);

    // 折叠关节不可控时目标跟随实测，使能后从原地保持，不会跳向旧目标
    if (!can_pitch1)
        pitch1.ref = pitch1.measure;

    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode_ == ECD_MODE) {
        // 换算到 imu 参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode_ == IMU_MODE) {
        // 换算到 ecd 参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 分配大小 yaw：大 yaw 直接设为总目标，小 yaw 补偿大 yaw 尚未转到的部分并限位
    yaw1.ref = yaw.ecd.ref;
    const float yaw2_ref = Angle(yaw.ecd.ref - yaw1.measure);
    yaw2.ref = std::clamp(yaw2_ref, fold_config_.yaw2.min, fold_config_.yaw2.max);
    const bool is_yaw2_limit = float(yaw2.ref) != yaw2_ref;

    // 俯仰与折叠关节软件限位
    float pitch2_ref = pitch.ecd.ref;
    if (config_.is_pitch_limit)
        pitch2_ref = std::clamp(pitch2_ref, config_.pitch_min, config_.pitch_max);
    const bool is_pitch2_limit = pitch2_ref != float(pitch.ecd.ref);
    pitch.ecd.ref = pitch2_ref;
    pitch1.ref = std::clamp(float(pitch1.ref), fold_config_.pitch1.min, fold_config_.pitch1.max);

    // 大 yaw 串级 PID，圆周误差；速度前馈抵消底盘旋转并叠加云台转速
    if (can_yaw1) {
        const float speed_ref = yaw_position_pid_.calculate(Angle(yaw1.ref - yaw1.measure)) - chassis_wr_ + yaw_speed_;
        yaw_speed_pid_.track(set_output(motor.yaw1, config_.yaw_output, yaw_speed_pid_.calculate(speed_ref - motor.yaw1.speed)));
    } else {
        motor.yaw1.set_current(0);
    }

    // 小 yaw 串级 PID，限位误差；速度前馈另扣除大 yaw 转速，到达限位时取消前馈
    if (can_yaw2) {
        float speed_ff = 0;
        if (!is_yaw2_limit)
            speed_ff = -chassis_wr_ + yaw_speed_ - motor.yaw1.speed;
        const float speed_ref = yaw2_position_pid_.calculate(yaw2.ref - yaw2.measure) + speed_ff;
        yaw2_speed_pid_.track(set_output(motor.yaw2, fold_config_.yaw2.output, yaw2_speed_pid_.calculate(speed_ref - motor.yaw2.speed)));
    } else {
        motor.yaw2.set_current(0);
    }

    // 俯仰串级 PID 叠加重力补偿，到达限位时取消速度前馈；PID 跟踪扣除补偿后的实际输出
    if (can_pitch2) {
        float speed_ff = 0;
        if (!is_pitch2_limit)
            speed_ff = pitch_speed_;
        const float speed_ref = pitch_position_pid_.calculate(pitch.ecd.ref - pitch.ecd.measure) + speed_ff;
        const float gravity = gravity_output(motor.pitch2, config_.pitch_output, fold_config_.pitch2_gravity, pitch.imu.measure);
        const float output = pitch_speed_pid_.calculate(speed_ref - motor.pitch2.speed) + gravity;
        pitch_speed_pid_.track(set_output(motor.pitch2, config_.pitch_output, output) - gravity);
    } else {
        motor.pitch2.set_current(0);
    }

    // 折叠关节串级 PID 叠加重力补偿
    if (can_pitch1) {
        const float speed_ref = pitch1_position_pid_.calculate(pitch1.ref - pitch1.measure);
        const float gravity = gravity_output(motor.pitch1, fold_config_.pitch1.output, fold_config_.pitch1_gravity, pitch1.measure);
        const float output = pitch1_speed_pid_.calculate(speed_ref - motor.pitch1.speed) + gravity;
        pitch1_speed_pid_.track(set_output(motor.pitch1, fold_config_.pitch1.output, output) - gravity);
    } else {
        motor.pitch1.set_current(0);
    }

    // 传递软件限位到 imu 参考系
    yaw.imu.ref = yaw1.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}

bool GimbalFold::check_motor(const Motor& motor, bool& is_ready, Pid& speed_pid, Pid& position_pid) const {
    const bool can_control = is_motors_enable_ && motor.is_connect;
    if (can_control && is_ready)
        return true;

    speed_pid.clear();
    position_pid.clear();
    is_ready = can_control;
    return false;
}

float GimbalFold::gravity_output(const Motor& motor, const PidOutput output, const Gravity& gravity, const float angle) {
    const float torque = gravity.torque * std::cos(angle + gravity.phase);
    if (output == CURRENT_OUTPUT)
        return torque / motor.config.drive.kt;
    return torque;
}
