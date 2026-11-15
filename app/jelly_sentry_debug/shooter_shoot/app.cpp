#include <algorithm>
#include "../../jelly_sentry/shooter.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 拨弹电机单独测试（灵足 EL05，CAN2 MIT 标准帧）：不经过发射机构，不要求摩擦轮开启
// 右手水平摇杆控制拨弹转速，满打对应 BULLET_FREQ；速度环使用整车 shoot_pid，输出力矩
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡
// 调试器观察 shoot.speed、shoot.torque、shoot.mode、shoot.fault，核对方向与 MIT 标准帧解析

static constexpr float BULLET_FREQ = 20.0f;    // 弹频，发/秒
static constexpr float BULLET_PER_ANGLE = 9.0f / rev; // 与整车发射机构一致，发/rad

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});
static Pid speed_pid(&shoot_pid);

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

extern "C" void app_main() {
    Timer loop_timer;
    uint32_t shoot_divider = 0;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        vt13.update();
        fsi6x.update();
        shoot.update();

        const bool is_enable = rc_enable();
        shoot.set_enable(is_enable);
        if (is_enable && shoot.is_connect) {
            const float speed = std::clamp(vt13.x + fsi6x.x, -1.0f, 1.0f) * BULLET_FREQ / BULLET_PER_ANGLE;
            shoot.set_torque(speed_pid.calculate(speed - shoot.speed));
            speed_pid.track(shoot.torque_ref);
        } else {
            speed_pid.clear();
            shoot.set_current(0);
        }

        // 控制计算保持 1 kHz，发送周期与整车 shoot_send_period_ms 共用
        if (++shoot_divider >= shoot_send_period_ms) {
            shoot_divider = 0;
            shoot.send();
        }
    }
}
