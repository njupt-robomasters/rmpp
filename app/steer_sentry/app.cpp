#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "minipc.hpp"
#include "shooter.hpp"
#include <tuple>

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
    yaw1.send(); // 大yaw 1000 Hz

    // 小yaw、拨弹、舵1/2 1000 Hz
    CanFrame frame_1ff{.id = 0x1FF, .size = 8};
    yaw2.write_can_frame(frame_1ff);
    shoot.write_can_frame(frame_1ff);
    s1.write_can_frame(frame_1ff);
    s2.write_can_frame(frame_1ff);
    Can(1).send(frame_1ff);

    // 第 3 帧轮流：底盘、舵3/4、裁判发送各约 333 Hz
    if (send_cnt % 3 == 0) { // 底盘 333 Hz
        CanFrame frame{.id = 0x200, .size = 8};
        for (M3508* w : {&w1, &w2, &w3, &w4})
            w->write_can_frame(frame);
        Can(1).send(frame);
    } else if (send_cnt % 3 == 1) { // 舵3/4 333 Hz
        CanFrame frame{.id = 0x2FF, .size = 8};
        s3.write_can_frame(frame);
        s4.write_can_frame(frame);
        Can(1).send(frame);
    } else { // 裁判发送 333 Hz
        referee_sender.send();
    }

    // CAN2
    pitch.send(); // pitch 1000 Hz
    rub1.send();  // 摩擦轮 1000 Hz
    rub2.send();

    // CDC：自瞄与导航共用一路，错拍各 200 Hz
    if (send_cnt % 5 == 0) // 自瞄 200 Hz
        aim.send();
    else if (send_cnt % 5 == 2) // 导航 200 Hz
        nav.send();
}

static void loop() {
    melody.update();
    vt13.set_in_game(referee.game_status.game_progress == Referee::GAMING); // 正赛中断联保持挡位
    vt13.update();

    referee_loop();
    minipc_loop();
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
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        // 负载监控
        static Monitor app_monitor;
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
