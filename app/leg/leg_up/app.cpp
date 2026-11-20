#include "app.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/unit.hpp"
#include "melody/music/c5d5g5.hpp"
#include "referee.hpp"
#include "shooter.hpp"
#include <cmath>

// 遥控器：键盘加减速度待实车整定
VT13 vt13({
    .uart = Uart(6),
    .vxy_max = 3.0f,
    .axy = INFINITY,
    .dxy = INFINITY,
    .wr_max = 360 * deg_s,
    .ar = INFINITY * deg_ss,
    .dr = INFINITY * deg_ss,
    .yaw_speed_max = 360 * deg_s,
    .pitch_speed_max = 360 * deg_s,
    .bullet_speed = 20.0f,
    .bullet_freq = 10.0f,
});

static void setup() {
    melody.play(C5D5G5);
    ui.init(referee_sender);
    Can(1).add_rx_callback(chassis_can_callback);
}

static void send() {
    static uint32_t send_cnt = 0;
    send_cnt++;

    // CAN1：奇拍底盘控制帧 500 Hz；偶拍超电 100 Hz，其余偶拍裁判发送（UI）至多 400 Hz
    if (send_cnt % 2 == 1) {
        CanFrame frame;
        chassis_write_can_frame(frame);
        Can(1).send(frame);
    } else if (send_cnt % 10 == 0) {
        super_cap.send();
    } else {
        referee_sender.send();
    }

    // CAN2：摩擦轮 100 Hz，其余拍 yaw、pitch 与拨弹
    if (send_cnt % 10 == 0) {
        rub1.send();
        rub2.send();
    } else {
        CanFrame frame1fe{.id = 0x1fe, .size = 8};
        yaw.write_can_frame(frame1fe);
        pitch.write_can_frame(frame1fe);
        Can(2).send(frame1fe);

        CanFrame frame200{.id = 0x200, .size = 8};
        shoot.write_can_frame(frame200);
        Can(2).send(frame200);
    }
}

static void loop() {
    melody.update();
    vt13.update();
    referee_loop();

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
