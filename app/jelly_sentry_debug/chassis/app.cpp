#include "../../jelly_sentry/chassis.hpp"
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <algorithm>

// 哨兵底盘测试：调试器修改下列变量控制平移和旋转，不依赖遥控器（分离模式，不跟随云台）
// 电机与 PID 使用整车 app/jelly_sentry/chassis.hpp
// 先设置目标速度，再将 is_enable 置 true；停止驱动时置 false
// 默认失能、零速度；volatile 保证运行中每次循环都读取调试器写入的值
// 功率按 120 W、缓冲能量 60 J 限制；确认前后、左右、旋转方向，不对则改 IS_WHEEL_INVERT 或单轮方向

volatile bool is_enable = false;
volatile float vx = 0; // 前正后负，m/s
volatile float vy = 0; // 左正右负，m/s
volatile float wr = 0; // 逆时针为正，rpm；首次测试建议 5 rpm

static constexpr float VXY_MAX = 1.0f;    // 平移最大速度，m/s
static constexpr float WR_MAX = 60 * rpm; // 旋转最大角速度，rad/s

static uint8_t chassis_divider = 0;

static void send_can_cmd() {
    // 与整车一致：底盘控制帧 500 Hz
    if (++chassis_divider < 2)
        return;
    chassis_divider = 0;
    CanFrame frame{.id = 0x200, .size = 8};
    w1.write_can_frame(frame);
    w2.write_can_frame(frame);
    w3.write_can_frame(frame);
    w4.write_can_frame(frame);
    Can(1).send(frame);
}

extern "C" void app_main() {
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_gimbal_yaw(0 * deg);
    super_cap.set_referee_limit(120);
    super_cap.set_referee_energy(60);

    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        chassis.set_enable(is_enable);
        chassis.set_speed(std::clamp(float(vx), -VXY_MAX, VXY_MAX), std::clamp(float(vy), -VXY_MAX, VXY_MAX), std::clamp(wr * rpm, -WR_MAX, WR_MAX));
        super_cap.update();
        chassis.set_power_ref(super_cap.power_chassis_ref);
        chassis.update();

        send_can_cmd();
    }
}
