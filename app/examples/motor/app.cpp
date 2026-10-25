#include "math/angle.hpp"
#include "motor/dm/dm4310.hpp"
#include "pid/pid.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include <algorithm>

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

static DM4310 motor({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x201},
    .drive = {.max_current = DM4310::MAX_CURRENT, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
});

static Pid::Config speed_pid_config = {
    .kp = 20 / (360 * rpm),
    .ki = 20 / (15 * deg),
    .max_i = 20,
    .max_out = 20,
};

static Pid::Config position_pid_config = {
    .kp = 10,
    .max_out = 60 * rpm,
};

static Pid speed_pid(&speed_pid_config);
static Pid position_pid(&position_pid_config);

static void current_loop(const float x) {
    motor.set_current(x * 20);
}

static void speed_loop(const float x) {
    const float speed = x * 60 * rpm;
    const float current = speed_pid.calculate(speed - motor.speed);
    motor.set_current(current);
}

static void position_loop(const float x) {
    const float position = x * 180 * deg;
    const float speed = position_pid.calculate(Angle(position - motor.position));
    const float current = speed_pid.calculate(speed - motor.speed);
    motor.set_current(current);
}

static void send_can_cmd() {
    // CanFrame frame{.id = 0x200, .size = 8};
    // motor.write_can_frame(frame);
    // Can(1).send(frame);
    motor.send();
}

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();

            vt13.update();
            fsi6x.update();
            motor.update();

            const bool is_enable = fsi6x.swa == FSi6X::DOWN || vt13.mode == VT13::N || vt13.mode == VT13::S;
            motor.set_enable(is_enable);

            if (is_enable) {
                // 两个 x 摇杆叠加后限幅到 ±1
                const float x = std::clamp(vt13.x + fsi6x.x, -1.0f, 1.0f);

                // 更换下面的调用即可切换电流、速度或位置环
                current_loop(x);
                // speed_loop(x);
                // position_loop(x);
            } else {
                motor.set_current(0);
                speed_pid.clear();
                position_pid.clear();
            }

            send_can_cmd();
        }
    }
}
