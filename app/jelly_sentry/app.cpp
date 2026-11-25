#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"
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

    // CAN2：pitch2 每拍，pitch1 每 4 拍（失能帧，保持反馈）
    pitch2.send();
    if (send_cnt % 4 == 0)
        pitch1.send();

    // CAN3：小 yaw 7025 每 2 拍（转矩与读角度各一次，合计 500 Hz）
    if (send_cnt % 2 == 0)
        yaw2.send();

    // CAN2：摩擦轮 VESC 每拍；拨弹 EL05 每 2 拍
    rub1.send();
    rub2.send();
    if (send_cnt % 2 == 0)
        shoot.send();

    // 自瞄上行 200 Hz
    if (send_cnt % 5 == 0)
        minipc_send();
}

static void loop() {
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
