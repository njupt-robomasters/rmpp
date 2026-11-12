#include "shooter.hpp"
#include "app.hpp"

static constexpr float BULLET_SPEED = 25.0f;         // 弹速，m/s
static constexpr float BULLET_FREQ = 10.0f;          // 弹频，发/秒
static constexpr float RUB_RELEASE_SPEED = 10 * rpm; // 摩擦轮关闭后低于此转速释放

void shooter_loop() {
    // N、S 挡使能，S 挡开摩擦轮，按住 trigger 时 pitch 摇杆作为弹频倍率，负值反转
    const bool is_enable = vt13.mode == VT13::N || vt13.mode == VT13::S;
    const bool is_rub = vt13.mode == VT13::S;
    const bool is_shoot = vt13.trigger;

    shooter.set_enable(is_enable);
    shooter.set_bullet_speed(is_rub ? BULLET_SPEED : 0.0f);
    shooter.set_bullet_freq(is_shoot ? BULLET_FREQ * vt13.pitch : 0.0f);
    shooter.update();

    // 摩擦轮关闭且低速时发零电流释放，避免 VESC 速度环在零速附近保持抖动
    if (!is_rub) {
        for (Vesc* rub : {&rub1, &rub2})
            if (std::fabs(rub->speed) < RUB_RELEASE_SPEED)
                rub->set_current(0);
    }

    // 发送 CAN 命令
    CanFrame frame{.id = 0x200, .size = 8};
    shoot.write_can_frame(frame);
    Can(2).send(frame);
    rub1.send();
    rub2.send();
}
