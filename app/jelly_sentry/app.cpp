#include "app.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "shooter.hpp"
#include <cmath>

// RGB 色彩循环
static constexpr float RGB_PERIOD = 1; // 循环周期，s
static float rgb_hue = 0;
static Timer rgb_timer;

static void setup() {
    // 大 yaw 零位从 KvStore 读取，未标定时为 0；标定方法见 gimbal.cpp 组合键
    yaw1.config.offset = kv_store.read("yaw_offset");
    melody.play(C5D5G5);
    ui.init(referee_sender);
}

static void send() {
    static uint32_t send_cnt = 0;
    send_cnt++;

    // 运行时可写周期的分频计数，周期 0、1 均为每拍发送
    static uint32_t pitch1_cnt = 0, pitch2_cnt = 0, yaw2_cnt = 0, shoot_cnt = 0;

    // CAN1：裁判发送每拍至多 1 帧
    referee_sender.send();

    // CAN1：底盘 C620 0x200 500 Hz，与 C620 回传对齐；需在 RoboMaster 调参助手中把回传频率设为 500 Hz
    if (send_cnt % 2 == 0) {
        CanFrame frame{.id = 0x200, .size = 8};
        for (M3508* w : {&w1, &w2, &w3, &w4})
            w->write_can_frame(frame);
        Can(1).send(frame);
    }

    // CAN1：大 yaw DM4310；超电 100 Hz，OFF、底盘失能与超电离线时也发送，保持反馈并支持重连
    yaw1.send();
    if (send_cnt % 10 == 0)
        super_cap.send();

    // CAN2：pitch1、pitch2 发送周期由 gimbal.hpp 的可写变量决定；pitch1 始终发送失能帧
    if (++pitch1_cnt >= pitch1_send_period_ms) {
        pitch1_cnt = 0;
        pitch1.send();
    }
    if (++pitch2_cnt >= pitch2_send_period_ms) {
        pitch2_cnt = 0;
        pitch2.send();
    }

    // CAN3：小 yaw 7025 转矩与读角度交替，各占发送频率的一半
    if (++yaw2_cnt >= yaw2_send_period_ms) {
        yaw2_cnt = 0;
        yaw2.send();
    }

    // CAN2：两路 P630，VESC 速度模式；失能时驱动发送零电流
    rub1.send();
    rub2.send();

    // CAN2：EL05 发送周期由 shooter.hpp 的可写变量决定
    if (++shoot_cnt >= shoot_send_period_ms) {
        shoot_cnt = 0;
        shoot.send();
    }
}

static void loop() {
    rgb_hue = std::fmod(rgb_hue + rgb_timer.restart() / RGB_PERIOD, 1.0f);
    rgb.set_hsv(rgb_hue, 1, 1);

    melody.update();
    vt13.update();
    attitude.update();
    referee_loop();

    // 上位机：姿态与比赛信息，弹速取裁判系统实测初速
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.update(attitude.yaw, attitude.pitch, attitude.roll);
    nav.update();

    chassis_loop();
    gimbal_loop();
    shooter_loop();

    send();
}

extern "C" void app_main() {
    setup();
    while (true) {
        // 1 kHz 更新
        static Timer app_timer;
        if (app_timer.elapsed() < 1 * ms)
            continue;
        app_timer.restart();

        // 负载监控
        static Monitor app_monitor;
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
