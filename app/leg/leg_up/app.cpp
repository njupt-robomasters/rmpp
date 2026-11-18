#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "shooter.hpp"

static void setup() {
    melody.play(C5D5G5);
    Can(1).add_rx_callback(chassis_can_callback);
}

static void loop() {
    melody.update();
    vt13.update();

    chassis_loop();
    gimbal_loop();
    shooter_loop();

    static uint32_t send_cnt = 0;
    send_cnt++;

    // CAN1 发送
    if (send_cnt % 10 == 0) { // 超级电容 100 Hz
        super_cap.send();
    } else if (send_cnt % 2 == 1) { // 底盘 500 Hz
        CanFrame frame;
        chassis_write_can_frame(frame);
        Can(1).send(frame);
    }

    // CAN2 发送
    if (send_cnt % 10 == 0) { // 摩擦轮 100 Hz
        rub1.send();
        rub2.send();
    } else {
        // yaw、pitch
        CanFrame frame1ff{.id = 0x1fe, .size = 8};
        yaw.write_can_frame(frame1ff);
        pitch.write_can_frame(frame1ff);
        Can(2).send(frame1ff);

        // shoot
        CanFrame frame200{.id = 0x200, .size = 8};
        shoot.write_can_frame(frame200);
        Can(2).send(frame200);
    }
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
