#include "../../jelly_sentry/shooter.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 哨兵双摩擦轮测试：VT13 S 挡或 FSi6X SWC 下开摩擦轮，目标转速按弹速与摩擦轮半径 3 cm 换算
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡
// P630 使用 CAN2 VESC 速度模式；发送暂时注释，保留状态接收与目标转速设置

static constexpr float BULLET_SPEED = 20.0f;          // 弹速，m/s
static constexpr float RUB_RADIUS = 3.0f * cm;        // 摩擦轮半径，m
static constexpr float RUB_SPEED = BULLET_SPEED / RUB_RADIUS; // 摩擦轮目标转速，rad/s

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
        for (Vesc* rub : {&rub1, &rub2}) {
            rub->update();
            rub->set_enable(rc_enable());
            rub->set_speed(is_rub ? RUB_SPEED : 0.0f);
            // rub->send(); // 调试阶段暂不发送
        }
    }
}
