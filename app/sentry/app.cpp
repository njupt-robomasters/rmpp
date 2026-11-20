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

// 设备状态：在线绿色，离线粉色
static RobotUi::Color status_color(const bool is_connect) {
    return is_connect ? RobotUi::GREEN : RobotUi::PINK;
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
    // F 键刷新 UI
    if (vt13.key.f)
        ui.init(referee_sender);

    ui.update(referee_sender, {
                                  .yaw_ecd = gimbal.yaw.ecd.measure, // 云台相对底盘角度
                                  .yaw = gimbal.yaw.imu.measure,
                                  .pitch = gimbal.pitch.imu.measure,
                                  .is_hurt = referee.hurt_timer.elapsed() < 0.2f,
                                  .hurt_dir = 0 * deg,
                                  .is_detect = aim.vision.is_detect,
                                  .cap = 0.0f,                                                      // 超级电容剩余能量比例
                                  .wr = std::isfinite(chassis.wr.measure) ? chassis.wr.measure : 0, // UI 数值字段在反馈未知时显示 0
                                  .bullet = vt13.config.bullet_freq,                                // 最高弹频
                                  .shoot = shoot.current,                                           // 拨弹电机电流
                                  .w1 = status_color(w1.is_connect),                                // 底盘
                                  .w2 = status_color(w2.is_connect),
                                  .w3 = status_color(w3.is_connect),
                                  .w4 = status_color(w4.is_connect),
                                  .s1 = status_color(s1.is_connect),
                                  .s2 = status_color(s2.is_connect),
                                  .s3 = status_color(s3.is_connect),
                                  .s4 = status_color(s4.is_connect),
                                  .yaw1 = status_color(yaw1.is_connect), // 云台
                                  .yaw2 = status_color(yaw2.is_connect),
                                  .pitch_motor = status_color(pitch.is_connect),
                                  .rub1 = rub_color(rub1, shooter.rub_speed[0]), // 发射机构
                                  .rub2 = rub_color(rub2, shooter.rub_speed[1]),
                                  .rub3 = rub_color(rub1, shooter.rub_speed[0]),
                                  .rub4 = rub_color(rub2, shooter.rub_speed[1]),
                                  .shooter = status_color(shoot.is_connect),
                                  .referee = status_color(referee.is_connect), // 裁判系统
                                  .aim = status_color(aim.is_connect_vision),  // 自瞄
                              });
}

static float cpu_usage = 0; // 主循环占用，SWD 观察

extern "C" void app_main() {
    // imu.start_calibration();
    yaw1.config.offset = kv_store.read("yaw_offset");
    melody.play(C5D5G5);
    ui.init(referee_sender);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            misc_loop();
            melody.update();
            referee.update();
            referee_sender.set_robot_id(referee.robot_status.robot_id);
            referee_sender.send();
            vt13.update();
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

            update_ui();
            send_can_cmd();
            const float running_time = timer.elapsed();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
