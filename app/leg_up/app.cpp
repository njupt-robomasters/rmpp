#include "app.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
#include "shooter.hpp"

static void setup() {
    melody.play(C5D5G5);
}

static void loop() {
    melody.update();
    vt13.update();

    gimbal_loop();
    shooter_loop();

    // CAN 发送
    static uint32_t send_cnt = 0;
    if (send_cnt++ % 10 == 0) { // 摩擦轮 100Hz
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
