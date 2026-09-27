#include "app.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"

static void send_can_cmd() {
    // CAN1
    ui.send_can_cmd();

    const int16_t cmd1 = w1.command_value();
    const int16_t cmd2 = w2.command_value();
    const int16_t cmd3 = w3.command_value();
    const int16_t cmd4 = w4.command_value();
    const int16_t cmd5 = yaw.command_value();
    const int16_t cmd6 = shoot.command_value();
    const int16_t cmd7 = 0;
    const int16_t cmd8 = 0;

    CanFrame frame{.id = 0x200, .size = 8};
    frame.data[0] = cmd1 >> 8;
    frame.data[1] = cmd1;
    frame.data[2] = cmd2 >> 8;
    frame.data[3] = cmd2;
    frame.data[4] = cmd3 >> 8;
    frame.data[5] = cmd3;
    frame.data[6] = cmd4 >> 8;
    frame.data[7] = cmd4;
    Can(1).send(frame);
    frame.data[0] = cmd5 >> 8;
    frame.data[1] = cmd5;
    frame.data[2] = cmd6 >> 8;
    frame.data[3] = cmd6;
    frame.data[4] = cmd7 >> 8;
    frame.data[5] = cmd7;
    frame.data[6] = cmd8 >> 8;
    frame.data[7] = cmd8;
    frame.id = 0x1FF;
    Can(1).send(frame);

    // CAN2
    pitch.send_can_cmd();
    rub1.send_can_cmd();
    rub2.send_can_cmd();
}

static void update_ui() {
    // 云台角度
    ui.yaw_ecd = gimbal.yaw.ecd.measure; // 云台相对底盘角度
    ui.yaw = gimbal.yaw.imu.measure;
    ui.pitch = gimbal.pitch.imu.measure;

    // 伤害方向
    ui.is_hurt = referee.hurt.is_hurt;
    ui.hurt_dir = referee.hurt.dir;

    // 自瞄识别
    ui.is_detect = mavlink.vision.is_detect;

    // 提示条
    ui.cap = 0.0f;                             // 超级电容剩余能量比例
    ui.wr = chassis.wr.measure;                // 底盘旋转速度
    if (referee.robot.type == Referee::HERO) { // 英雄显示弹速
        ui.bullet = shooter.config.bullet_speed;
        ui.bullet_is_speed = true;
    } else if (referee.robot.type == Referee::INFANTRY) { // 步兵显示弹频
        ui.bullet = shooter.config.bullet_freq;
        ui.bullet_is_speed = false;
    }
    ui.shoot = shooter.shoot_current; // 拨弹电机电流

    // 机器人状态
    chassis.update_ui(ui);                                           // 底盘
    gimbal.update_ui(ui);                                            // 云台
    shooter.update_ui(ui);                                           // 发射机构
    ui.robot.referee = referee.is_connect ? Ui::GREEN : Ui::PINK;    // 裁判系统
    ui.robot.aim = mavlink.is_connect_vision ? Ui::GREEN : Ui::PINK; // 自瞄

    ui.update();
}

extern "C" void app_main() {
    // gimbal.imu.calibrate();
    gimbal.load_yaw_offset(storage);
    buzzer.play(Buzzer::C5D5G5);

    [[maybe_unused]] static float cpu_usage{};
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rgb.update();
            buzzer.update();
            rc.update();
            referee.set_yaw(gimbal.yaw.ecd.measure, gimbal.yaw.imu.measure);
            referee.update();
            mavlink.imu = {.yaw = gimbal.imu.yaw, .pitch = gimbal.imu.pitch, .roll = gimbal.imu.roll};
            mavlink.referee = {
                .is_red = referee.game.is_red,
                .game_progress = referee.game.game_progress,
                .stage_remain_time = referee.game.stage_remain_time,
                .bullet_speed = referee.shooter.bullet_speed,
            };
            mavlink.update();

            chassis_loop();
            gimbal_loop();
            shooter_loop();

            if (rc.is_refresh_ui)
                ui.init();
            update_ui();
            send_can_cmd();
            const float running_time = timer.elapsed();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
