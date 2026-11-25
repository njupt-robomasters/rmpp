#include "referee.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

static constexpr float MOTOR_OVERHEAT = 80; // 电机温度报警阈值，℃
static constexpr float HURT_TIMEOUT = 1;    // 受击方向显示时长，s

// 受击方向，收到新的 0x0206 时按装甲 ID 换算到世界系并锁存
static uint32_t last_hurt_sequence = 0;
static bool has_hurt = false;
static Angle hurt_dir;
static Timer hurt_timer; // 上次受击后重新开始

// 下板：断联粉色、未使能白色、倒地青色、正常绿色
static RobotUi::Color down_color() {
    if (!leg_protocol.is_connect)
        return RobotUi::PINK;
    if (!leg_protocol.received.is_enable)
        return RobotUi::WHITE;
    if (leg_protocol.received.is_fallen)
        return RobotUi::CYAN;
    return RobotUi::GREEN;
}

// 下板电机：离线粉色、达妙错误黄色、过热橙色、失能白色、正常绿色
static RobotUi::Color down_motor_color(const LegDownFrame::MotorState state) {
    switch (state) {
    case LegDownFrame::MotorState::NORMAL:
        return leg_protocol.received.is_enable ? RobotUi::GREEN : RobotUi::WHITE;
    case LegDownFrame::MotorState::OVERHEAT:
        return RobotUi::ORANGE;
    case LegDownFrame::MotorState::ERROR:
        return RobotUi::YELLOW;
    case LegDownFrame::MotorState::OFFLINE:
        return RobotUi::PINK;
    }
    return RobotUi::WHITE;
}

static RobotUi::Color motor_color(const Motor::Status status) {
    switch (status) {
    case Motor::Status::OFFLINE:
        return RobotUi::PINK;
    case Motor::Status::DISABLED:
        return RobotUi::WHITE;
    case Motor::Status::OVERHEAT:
        return RobotUi::ORANGE;
    case Motor::Status::NORMAL:
        return RobotUi::GREEN;
    }
    return RobotUi::WHITE;
}

void referee_loop() {
    referee.update();
    referee_sender.set_robot_id(referee.robot_status.robot_id);

    // F 键按下时刷新 UI
    static bool last_f = false;
    if (vt13.key.f && !last_f)
        ui.init(referee_sender);
    last_f = vt13.key.f;

    // 受击方向：仅统计装甲受弹丸攻击；装甲 ID 0~3 依次位于底盘前方逆时针 0°、90°、180°、270°
    // 底盘世界航向 = 云台世界 yaw − yaw 电机相对底盘角
    if (referee.hurt_sequence != last_hurt_sequence) {
        last_hurt_sequence = referee.hurt_sequence;
        const auto [armor_id, HP_deduction_reason] = referee.hurt_data;
        if (HP_deduction_reason == 0 && armor_id <= 3) {
            hurt_dir = armor_id * 90 * deg + gimbal.yaw.angle_measure - yaw.angle;
            hurt_timer.restart();
            has_hurt = true;
        }
    }

    // 云台角度：编码器角度用于朝向弧，IMU 角度用于数值显示
    ui.yaw_ecd = yaw.angle;
    ui.yaw_angle = gimbal.yaw.angle_measure;
    ui.pitch_angle = gimbal.pitch.angle_measure;

    // 受击方向：限时显示，换算到云台坐标系
    ui.is_hurt = has_hurt && hurt_timer.elapsed() < HURT_TIMEOUT;
    ui.hurt_dir = hurt_dir - gimbal.yaw.angle_measure;

    // 仪表读数：超电能量占比、下板实测航向角速度、VT13 设定弹频和拨弹实测电流
    ui.super_cap = super_cap.energy_ratio();
    ui.chassis_wr = leg_protocol.received.wr;
    ui.bullet_freq = vt13.config.bullet_freq;
    ui.shoot_output = shoot.current;

    // 下板：yaw1 显示通信、使能与倒地，w1~w4 为关节，s1~s4 为轮；断联时电机全部离线
    ui.yaw1 = down_color();
    if (leg_protocol.is_connect) {
        ui.w1 = down_motor_color(leg_protocol.received.motor[0]);         // 左前
        ui.w2 = down_motor_color(leg_protocol.received.motor[1]);         // 左后
        ui.w3 = down_motor_color(leg_protocol.received.motor[2]);         // 右后
        ui.w4 = down_motor_color(leg_protocol.received.motor[3]);         // 右前
        ui.s1 = ui.s2 = down_motor_color(leg_protocol.received.motor[4]); // 左轮
        ui.s3 = ui.s4 = down_motor_color(leg_protocol.received.motor[5]); // 右轮
    } else {
        ui.w1 = ui.w2 = ui.w3 = ui.w4 = ui.s1 = ui.s2 = ui.s3 = ui.s4 = RobotUi::PINK;
    }

    // 云台：yaw 电机画在 yaw2，pitch 电机画在 pitch1，pitch2 未使用保持白色
    ui.yaw2 = motor_color(yaw.status(MOTOR_OVERHEAT));
    ui.pitch1 = motor_color(pitch.status(MOTOR_OVERHEAT));

    // 发射机构：单级摩擦轮的同侧两个图元共用状态
    ui.rub_left_stage1 = ui.rub_left_stage2 = motor_color(rub1.status(MOTOR_OVERHEAT, shooter.bullet_speed.ref != 0));
    ui.rub_right_stage1 = ui.rub_right_stage2 = motor_color(rub2.status(MOTOR_OVERHEAT, shooter.bullet_speed.ref != 0));
    ui.shoot = motor_color(shoot.status(MOTOR_OVERHEAT));

    // 超电状态；轮腿未接自瞄，图元保持白色，准星圆不显示
    if (!super_cap.is_connect)
        ui.cap = RobotUi::PINK;
    else if (super_cap.mode == SuperCap::OFF || super_cap.mode == SuperCap::CHARGE)
        ui.cap = RobotUi::WHITE;
    else if (super_cap.mode == SuperCap::BOOST)
        ui.cap = super_cap.is_cap_voltage_low() ? RobotUi::CYAN : RobotUi::GREEN;

    ui.update(referee_sender);
}
