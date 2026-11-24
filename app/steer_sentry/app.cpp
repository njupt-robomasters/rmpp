#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "shooter.hpp"
#include <cmath>
#include <tuple>

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

    // CAN1：C 板 3 个发送邮箱、无软件队列，每拍至多 3 帧
    // 大 yaw 与 0x1FF（小 yaw、拨弹、舵 1/2）每拍发送；第 3 帧由 0x200（轮）、0x2FF（舵 3/4）、裁判发送轮流，各约 333 Hz
    yaw1.send();

    CanFrame frame_1ff{.id = 0x1FF, .size = 8};
    yaw2.write_can_frame(frame_1ff);
    shoot.write_can_frame(frame_1ff);
    s1.write_can_frame(frame_1ff);
    s2.write_can_frame(frame_1ff);
    Can(1).send(frame_1ff);

    if (send_cnt % 3 == 0) {
        CanFrame frame{.id = 0x200, .size = 8};
        for (M3508* w : {&w1, &w2, &w3, &w4})
            w->write_can_frame(frame);
        Can(1).send(frame);
    } else if (send_cnt % 3 == 1) {
        CanFrame frame{.id = 0x2FF, .size = 8};
        s3.write_can_frame(frame);
        s4.write_can_frame(frame);
        Can(1).send(frame);
    } else {
        referee_sender.send();
    }

    // CAN2：pitch、两路 VESC 摩擦轮
    pitch.send();
    rub1.send();
    rub2.send();

    if (send_cnt % 5 == 0) {
        aim.set_imu(attitude.yaw, attitude.pitch, attitude.roll);
        aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
        aim.send();
    }
}

static void loop() {
    rgb_hue = std::fmod(rgb_hue + rgb_timer.restart() / RGB_PERIOD, 1.0f);
    rgb.set_hsv(rgb_hue, 1, 1);

    melody.update();
    vt13.update();
    attitude.update();
    aim.update();
    referee_loop();
    nav.update();

    chassis_loop();
    const auto [is_detect, target_yaw, target_pitch, distance, is_fire] = [] {
        const IrqLock lock;
        return std::tuple{aim.is_detect, aim.yaw, aim.pitch, aim.distance, aim.is_fire};
    }();
    gimbal_loop(is_detect, target_yaw, target_pitch, distance);
    shooter_loop(is_fire);

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
