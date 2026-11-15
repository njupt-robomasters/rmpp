#include "bsp/monitor.hpp"
#include "math/angle.hpp"
#include "motor/gm6020.hpp"
#include "pid/pid.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

static constexpr float MAX_SPEED = 60 * rpm;
static float speed_ref;

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

static GM6020 motor({
    .can = Can(2),
    .tx_id = 0x1fe,
    .rx_id = 0x205,
    .is_invert = true,
});

static Pid::Config speed_pid_config = {
    .kp = 0,
    .ki = 0,
    .max_i = GM6020::MAX_CURRENT,
    .max_out = GM6020::MAX_CURRENT,
};

static Pid speed_pid(&speed_pid_config);

static void setup() {}

static void loop() {
    static Timer loop_timer;
    const float dt = loop_timer.restart();

    vt13.update();
    fsi6x.update();
    motor.update();

    const bool is_enable = fsi6x.swa == FSi6X::DOWN || vt13.mode == VT13::N || vt13.mode == VT13::S;
    motor.set_enable(is_enable);

    if (is_enable) {
        speed_ref = MAX_SPEED * (vt13.x + fsi6x.x);
        const float current = speed_pid.calculate(dt, speed_ref - motor.speed);
        motor.set_current(current);
    } else {
        motor.set_current(0);
        speed_pid.clear();
    }

    // 发送 CAN 命令
    CanFrame frame{.id = 0x1fe, .size = 8};
    motor.write_can_frame(frame);
    Can(2).send(frame);
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
