#include "app.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "misc/music/c5d5g5.hpp"

static void send_can_cmd() {
    // CAN1
    ui.send_can_cmd();

    CanFrame chassis_frame{.id = 0x200, .size = 8};
    w1.write_can_frame(chassis_frame);
    w2.write_can_frame(chassis_frame);
    w3.write_can_frame(chassis_frame);
    w4.write_can_frame(chassis_frame);
    Can(1).send(chassis_frame);

    CanFrame gimbal_frame{.id = 0x1FF, .size = 8};
    yaw.write_can_frame(gimbal_frame);
    shoot.write_can_frame(gimbal_frame);
    Can(1).send(gimbal_frame);

    // CAN2
    pitch.send();
    rub1.send();
    rub2.send();
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
    // 弹速或弹频由 shooter_loop 写入
    ui.cap = 0.0f;                    // 超级电容剩余能量比例
    ui.wr = chassis.wr.measure;       // 底盘旋转速度
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
    // gimbal.imu.start_calibration();
    gimbal.load_yaw_offset(storage);
    buzzer.play(C5D5G5);

    [[maybe_unused]] static float cpu_usage{};
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            misc_loop();
            buzzer.update();
            vt13.update();
            fsi6x.update();
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

            if (vt13.key.f) // F 键刷新 UI
                ui.init();
            update_ui();
            send_can_cmd();
            const float running_time = timer.elapsed();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
