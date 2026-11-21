#include "referee.hpp"
#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"
#include <algorithm>
#include <cmath>

static constexpr float MOTOR_OVERHEAT = 80; // 电机温度报警阈值，℃
static constexpr float HURT_TIMEOUT = 1;    // 受击方向显示时长，s

// 受击方向，收到新的 0x0206 时按装甲 ID 换算到 IMU 参考系并锁存
static uint32_t last_hurt_sequence = 0;
static bool has_hurt = false;
static Angle hurt_dir;
static Timer hurt_timer; // 上次受击后重新开始

// 设备状态：在线绿色，离线粉色
static RobotUi::Color status_color(const bool is_connect) {
    return is_connect ? RobotUi::GREEN : RobotUi::PINK;
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
    if (referee.hurt_sequence != last_hurt_sequence) {
        last_hurt_sequence = referee.hurt_sequence;
        const auto [armor_id, HP_deduction_reason] = referee.hurt_data;
        if (HP_deduction_reason == 0 && armor_id <= 3) {
            hurt_dir = armor_id * 90 * deg + gimbal.yaw.imu_minus_ecd;
            hurt_timer.restart();
            has_hurt = true;
        }
    }

    // 哨兵无舵轮，s1~s4 保持白色；UI 只有一个 pitch 状态位，任一 pitch 电机断联都显示为断联
    ui.yaw_ecd = gimbal.yaw.ecd.measure; // 云台相对底盘角度
    ui.yaw_angle = gimbal.yaw.imu.measure;
    ui.pitch_angle = gimbal.pitch.imu.measure;
    ui.is_hurt = has_hurt && hurt_timer.elapsed() < HURT_TIMEOUT;
    ui.hurt_dir = hurt_dir - gimbal.yaw.imu.measure; // 相对云台
    ui.is_aim_detect = aim.vision.is_detect;
    ui.super_cap = super_cap.energy_ratio();
    if (!super_cap.is_connect)
        ui.cap = RobotUi::PINK;
    else if (super_cap.mode == SuperCap::CHARGE)
        ui.cap = RobotUi::CYAN;
    else if (super_cap.mode == SuperCap::BOOST)
        ui.cap = super_cap.is_cap_voltage_low() ? RobotUi::CYAN : RobotUi::GREEN;
    else
        ui.cap = RobotUi::WHITE;
    ui.chassis_wr = std::isfinite(chassis.wr.measure) ? chassis.wr.measure : 0; // 反馈未知时显示 0
    ui.bullet_freq = vt13.config.bullet_freq;                                   // 最高弹频
    ui.shoot_output = shoot.torque;                                             // 拨弹电机力矩
    ui.w1 = motor_color(w1.status(MOTOR_OVERHEAT));                             // 底盘
    ui.w2 = motor_color(w2.status(MOTOR_OVERHEAT));
    ui.w3 = motor_color(w3.status(MOTOR_OVERHEAT));
    ui.w4 = motor_color(w4.status(MOTOR_OVERHEAT));
    ui.yaw1 = motor_color(yaw1.status(MOTOR_OVERHEAT)); // 云台
    ui.yaw2 = motor_color(yaw2.status(MOTOR_OVERHEAT));
    ui.pitch1 = motor_color(pitch1.status(MOTOR_OVERHEAT));
    ui.pitch2 = motor_color(pitch2.status(MOTOR_OVERHEAT));
    ui.rub_left_stage1 = ui.rub_left_stage2 = motor_color(rub1.status(MOTOR_OVERHEAT, shooter.bullet_speed.ref != 0)); // 发射机构
    ui.rub_right_stage1 = ui.rub_right_stage2 = motor_color(rub2.status(MOTOR_OVERHEAT, shooter.bullet_speed.ref != 0));
    ui.shoot = motor_color(shoot.status(MOTOR_OVERHEAT));
    ui.aim = status_color(aim.is_connect_vision); // 自瞄
    ui.update(referee_sender);
}
