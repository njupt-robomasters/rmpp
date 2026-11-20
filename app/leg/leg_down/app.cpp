#include "app.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "melody/music/c5d5g5.hpp"

// 发送拍数，send() 末尾递增；偶拍发送左腿并计算底盘，奇拍发送右腿
static uint32_t send_cnt = 0;

static void setup() {
    melody.play(C5D5G5);
    Can(1).add_rx_callback(chassis_can_callback);
}

static void send() {
    // CAN1 左右关节交替发送，每个关节 500 Hz；偶拍与奇拍各 3 帧，匹配 3 个发送邮箱
    if (send_cnt % 2 == 0) {
        left_front.send();
        left_back.send();
        CanFrame frame{.id = 0x200, .size = 8};
        left_wheel.write_can_frame(frame);
        right_wheel.write_can_frame(frame);
        Can(1).send(frame);
    } else {
        right_back.send();
        right_front.send();

        // 第 3 帧：底盘状态帧 0x003 与下板裁判帧 0x00 轮流，各 250 Hz
        if (send_cnt % 4 == 1) {
            CanFrame frame;
            chassis_write_can_frame(frame);
            Can(1).send(frame);
        } else {
            referee_relay.send();
        }
    }
    send_cnt++;
}

static void loop() {
    melody.update();
    fsi6x.update();
    attitude.update();

    // 底盘只在发送左腿的一拍计算（500 Hz），保证两侧关节反馈都已刷新
    if (send_cnt % 2 == 0)
        chassis_loop();

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
