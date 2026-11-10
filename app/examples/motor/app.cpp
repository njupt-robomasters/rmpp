#include "bsp/monitor.hpp"
#include "math/angle.hpp"
#include "motor/m3508.hpp"
#include "pid/pid.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include <algorithm>

static constexpr float MAX_CURRENT = M3508::MAX_CURRENT;
static constexpr float MAX_SPEED = 60 * rpm;
static constexpr float MAX_POSITION = 180 * deg;

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

static M3508 motor({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x201},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
});

static Pid::Config speed_pid_config = {
    .kp = 1,
    .ki = 0,
    .max_i = MAX_CURRENT,
    .max_out = MAX_CURRENT,
};

static Pid::Config position_pid_config = {
    .kp = 1,
    .max_out = MAX_SPEED,
};

static Pid speed_pid(&speed_pid_config);
static Pid position_pid(&position_pid_config);

static void current_loop(const float x) {
    motor.set_current(x * MAX_CURRENT);
}

static void speed_loop(const float x) {
    const float speed = x * MAX_SPEED;
    const float current = speed_pid.calculate(speed - motor.speed);
    motor.set_current(current);
}

static void position_loop(const float x) {
    const float position = x * MAX_POSITION;
    const float speed = position_pid.calculate(Angle(position - motor.position));
    const float current = speed_pid.calculate(speed - motor.speed);
    motor.set_current(current);
}

static void setup() {}

static void loop() {
    vt13.update();
    fsi6x.update();
    motor.update();

    const bool is_enable = fsi6x.swa == FSi6X::DOWN || vt13.mode == VT13::N || vt13.mode == VT13::S;
    motor.set_enable(is_enable);

    if (is_enable) {
        // 两个 x 摇杆叠加后限幅到 ±1
        const float x = std::clamp(vt13.x + fsi6x.x, -1.0f, 1.0f);

        // 更换下面的调用即可切换电流、速度或位置环
        // current_loop(x);
        speed_loop(x);
        // position_loop(x);
    } else {
        motor.set_current(0);
        speed_pid.clear();
        position_pid.clear();
    }

    // 发送 CAN 命令
    static CanFrame frame{.id = 0x200, .size = 8};
    motor.write_can_frame(frame);
    Can(1).send(frame);
    // motor.send();
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
