#include "bsp/monitor.hpp"
#include "motor/m3508.hpp"
#include "pid/pid.hpp"
#include "rc/vt13.hpp"
#include "referee/referee_sender.hpp"
#include <algorithm>
#include <cmath>

static constexpr float MAX_SPEED = 60 * rpm;
static constexpr float SPEED_TOLERANCE = 5 * rpm;
static constexpr float MOTOR_OVERHEAT = 80;
static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s

static VT13 vt13({.uart = Uart(6)});

static M3508 motor({
    .can = Can(1),
    .tx_id = 0x200,
    .rx_id = 0x201,
    .is_invert = false,
});

static Pid::Config speed_pid_config = {
    .kp = 0,
    .ki = 0,
    .max_i = M3508::MAX_CURRENT,
    .max_out = M3508::MAX_CURRENT,
};

// 观察量：转速目标，rad/s
static float speed_ref = 0;
static volatile RefereeSender::Color motor_status_color = RefereeSender::WHITE;

static Pid speed_pid(&speed_pid_config);
static Timer update_timer;

static void setup() {}

static void send() {
    CanFrame frame{.id = 0x200, .size = 8};
    motor.write_can_frame(frame);
    Can(1).send(frame);
}

static void loop() {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    vt13.update();
    motor.update();

    // VT13 N、S 挡使能，摇杆 rc.x 控制转速
    motor.set_enable(vt13.is_enable);
    if (vt13.is_enable) {
        speed_ref = MAX_SPEED * vt13.rc.x;
        motor.set_current(speed_pid.calculate(dt, speed_ref - motor.speed));
    } else {
        speed_ref = 0;
        speed_pid.clear();
        motor.set_current(0);
    }

    switch (motor.status(MOTOR_OVERHEAT, speed_ref != 0)) {
    case Motor::Status::OFFLINE:
        motor_status_color = RefereeSender::PINK;
        break;
    case Motor::Status::DISABLED:
        motor_status_color = RefereeSender::WHITE;
        break;
    case Motor::Status::OVERHEAT:
        motor_status_color = RefereeSender::ORANGE;
        break;
    case Motor::Status::NORMAL:
        motor_status_color = std::fabs(speed_ref - motor.speed) > SPEED_TOLERANCE ? RefereeSender::CYAN : RefereeSender::GREEN;
        break;
    }

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
