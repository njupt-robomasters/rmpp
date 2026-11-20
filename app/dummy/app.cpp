#include "app.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "referee.hpp"

static void setup() {
    melody.play(C5D5G5);
}

static void loop() {
    melody.update();
    fsi6x.update();

    referee_loop();
    chassis_loop();
    gimbal_loop();
}

static void send() {
    static uint32_t send_cnt = 0;
    send_cnt++;

    // CAN1：底盘 0x200 每拍，超电 100 Hz
    CanFrame frame{.id = 0x200, .size = 8};
    for (X630* w : {&w1, &w2, &w3, &w4})
        w->write_can_frame(frame);
    Can(1).send(frame);
    if (send_cnt % 10 == 0)
        super_cap.send();

    // CAN2：yaw RMDL7025 两种报文合计应答上限 500 Hz
    if (send_cnt % 2 == 0)
        yaw.send();
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
        send();
        app_monitor.end();
    }
}
