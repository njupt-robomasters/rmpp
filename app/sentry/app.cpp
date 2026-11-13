#include "app.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "melody/music/c5d5g5.hpp"
#include <cmath>

static void send_can_cmd() {
    // CAN1

    CanFrame wheel_frame{.id = 0x200, .size = 8};
    w1.write_can_frame(wheel_frame);
    w2.write_can_frame(wheel_frame);
    w3.write_can_frame(wheel_frame);
    w4.write_can_frame(wheel_frame);
    Can(1).send(wheel_frame);

    CanFrame frame_1ff{.id = 0x1FF, .size = 8};
    yaw2.write_can_frame(frame_1ff);
    shoot.write_can_frame(frame_1ff);
    s1.write_can_frame(frame_1ff);
    s2.write_can_frame(frame_1ff);
    Can(1).send(frame_1ff);

    CanFrame frame_2ff{.id = 0x2FF, .size = 8};
    s3.write_can_frame(frame_2ff);
    s4.write_can_frame(frame_2ff);
    Can(1).send(frame_2ff);

    yaw1.send();

    // CAN2
    pitch.send();
    rub1.send();
    rub2.send();
}

// 摩擦轮状态：离线粉色，偏离目标弹速超过 1 m/s 橙色
static RobotUi::Color rub_color(const Motor& motor, const float speed) {
    if (!motor.is_connect)
        return RobotUi::PINK;
    if (std::fabs(shooter.bullet_speed.ref - speed) > 1)
        return RobotUi::ORANGE;
    return RobotUi::GREEN;
}

static void update_ui() {
    // 云台角度
    ui.yaw_ecd = gimbal.yaw.ecd.measure; // 云台相对底盘角度
    ui.yaw = gimbal.yaw.imu.measure;
    ui.pitch = gimbal.pitch.imu.measure;

    // 伤害方向
    ui.is_hurt = referee.hurt_timer.elapsed() < 0.2f;
    ui.hurt_dir = 0 * deg;

    // 自瞄识别
    ui.is_detect = aim.vision.is_detect;

    // 提示条
    // 弹速或弹频由 shooter_loop 写入
    ui.cap = 0.0f;              // 超级电容剩余能量比例
    ui.wr = chassis.wr.measure; // 底盘旋转速度
    ui.shoot = shoot.current;   // 拨弹电机电流

    // 机器人状态
    ui.robot.w1 = w1.is_connect ? RobotUi::GREEN : RobotUi::PINK; // 底盘
    ui.robot.w2 = w2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w3 = w3.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w4 = w4.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s1 = s1.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s2 = s2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s3 = s3.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.s4 = s4.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.yaw1 = yaw1.is_connect ? RobotUi::GREEN : RobotUi::PINK; // 云台
    ui.robot.yaw2 = yaw2.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.pitch = pitch.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.rub1 = ui.robot.rub3 = rub_color(rub1, shooter.rub_speed[0]); // 发射机构
    ui.robot.rub2 = ui.robot.rub4 = rub_color(rub2, shooter.rub_speed[1]);
    ui.robot.shoot = shoot.is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.referee = referee.is_connected ? RobotUi::GREEN : RobotUi::PINK; // 裁判系统
    ui.robot.aim = aim.is_connect_vision ? RobotUi::GREEN : RobotUi::PINK;    // 自瞄

    ui.update();
}

static float cpu_usage = 0; // 主循环占用，SWD 观察

extern "C" void app_main() {
    // imu.start_calibration();
    gimbal.load_yaw_offset(kv_store);
    melody.play(C5D5G5);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            misc_loop();
            melody.update();
            referee.update();
            vt13.update();
            fsi6x.update();
            aim.imu = {.yaw = attitude.yaw, .pitch = attitude.pitch, .roll = attitude.roll};
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
