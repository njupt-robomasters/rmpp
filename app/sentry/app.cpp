#include "app.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "melody/music/c5d5g5.hpp"

static void send_can_cmd() {
    // CAN1

    CanFrame chassis_frame{.id = 0x200, .size = 8};
    w1.write_can_frame(chassis_frame);
    w2.write_can_frame(chassis_frame);
    Can(1).send(chassis_frame);

    CanFrame gimbal_frame{.id = 0x1FF, .size = 8};
    yaw2.write_can_frame(gimbal_frame);
    shoot.write_can_frame(gimbal_frame);
    s1.write_can_frame(gimbal_frame);
    s2.write_can_frame(gimbal_frame);
    Can(1).send(gimbal_frame);

    yaw1.send();

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
    ui.is_hurt = referee.hurt_received.elapsed() < 0.2f;
    ui.hurt_dir = 0 * deg;

    // 自瞄识别
    ui.is_detect = aim.vision.is_detect;

    // 提示条
    // 弹速或弹频由 shooter_loop 写入
    ui.cap = 0.0f;                    // 超级电容剩余能量比例
    ui.wr = chassis.wr.measure;       // 底盘旋转速度
    ui.shoot = shooter.shoot_current; // 拨弹电机电流

    // 机器人状态
    chassis.update_ui(ui);                                          // 底盘
    gimbal.update_ui(ui);                                           // 云台
    shooter.update_ui(ui);                                          // 发射机构
    ui.robot.referee = referee.is_connected ? RobotUi::GREEN : RobotUi::PINK; // 裁判系统
    ui.robot.aim = aim.is_connect_vision ? RobotUi::GREEN : RobotUi::PINK;    // 自瞄

    ui.update();
}

extern "C" void app_main() {
    // gimbal.attitude.start_calibration();
    gimbal.load_yaw_offset(storage);
    melody.play(C5D5G5);

    [[maybe_unused]] static float cpu_usage{};
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            misc_loop();
            melody.update();
            referee.update();
            vt13.update();
            fsi6x.update();
            aim.imu = {.yaw = gimbal.attitude.yaw, .pitch = gimbal.attitude.pitch, .roll = gimbal.attitude.roll};
            aim.referee = {
                .is_red = referee.robot_status.robot_id < 100,
                .game_progress = static_cast<Referee::GameProgress>(referee.game_status.game_progress),
                .stage_remain_time = static_cast<float>(referee.game_status.stage_remain_time),
                .bullet_speed = referee.robot_status.bullet_speed_limit,
            };
            aim.update();
            nav.update();

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
