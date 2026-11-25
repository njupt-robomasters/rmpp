#include "app.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "minipc.hpp"
#include "shooter.hpp"

static void setup() {
    // 大 yaw 零位从 KvStore 读取，未标定时为 0；标定方法见 gimbal.cpp 组合键
    yaw1.config.offset = kv_store.read("yaw_offset");
    melody.play(C5D5G5);
    ui.init(referee_sender);
}

static void send() {
    static uint32_t send_cnt = 0;
    send_cnt++;

    // CAN1
    referee_sender.send();   // 裁判发送每拍至多 1 帧
    if (send_cnt % 2 == 0) { // 底盘 500 Hz
        CanFrame frame{.id = 0x200, .size = 8};
        for (const M3508* w : {&w1, &w2, &w3, &w4})
            w->write_can_frame(frame);
        Can(1).send(frame);
    }
    yaw1.send();            // 大yaw 1000 Hz
    if (send_cnt % 10 == 0) // 超电 100 Hz
        super_cap.send();

    // CAN2
    if (send_cnt % 4 == 0) // pitch1 250Hz
        pitch1.send();
    pitch2.send();         // pitch2 1000 Hz
    if (send_cnt % 2 == 0) // shoot 500 Hz
        shoot.send();
    rub1.send(); // 摩擦轮 1000 Hz
    rub2.send();

    // CAN3
    if (send_cnt % 2 == 0) // 小yaw 500Hz
        yaw2.send();

    // 自瞄上行 200 Hz
    if (send_cnt % 5 == 0)
        minipc_send();
}

static void loop() {
    melody.update();
    vt13.update();
    attitude.update();

    referee_loop();
    minipc_loop();

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
