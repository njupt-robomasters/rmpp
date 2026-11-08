#include "../../jelly_sentry/shooter.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 哨兵发射机构测试：电机与 PID 使用整车 app/jelly_sentry/shooter.hpp
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；VT13 S 挡或 FSi6X SWC 下开摩擦轮
// 右手水平摇杆作为弹频倍率，负值反转拨弹；摩擦轮未开时不拨弹
// 摩擦轮 P630 驱动待写，占位对象不发送；拨弹 EL05 可单独测试卡弹反转

static constexpr float BULLET_SPEED = 20.0f; // 弹速，m/s
static constexpr float BULLET_FREQ = 20.0f;  // 弹频，发/秒

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

extern "C" void app_main() {
    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        vt13.update();
        fsi6x.update();

        const bool is_rub = vt13.mode == VT13::S || fsi6x.swc == FSi6X::DOWN;
        shooter.set_enable(rc_enable());
        shooter.set_bullet_speed(is_rub ? BULLET_SPEED : 0.0f);
        shooter.set_bullet_freq((vt13.x + fsi6x.x) * BULLET_FREQ);
        shooter.update();

        shoot.send();
    }
}
