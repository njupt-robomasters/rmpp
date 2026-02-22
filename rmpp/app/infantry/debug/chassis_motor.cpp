#include "../led.hpp"
#include "../rc.hpp"
#include "../chassis.hpp"

static constexpr UnitFloat MAX_SPEED = 60 * rpm;

static M3508& w = w1;
static PID::config_t speed_pid = {
    .kp = (20 * A) / (120 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A,
};

void send_can_cmd() {
    const int16_t cmd1 = w.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(w.config.can_port, 0x200, data, 8);
}

void setup() {
    BSP::Init();

    w.config.control_mode = Motor::SPEED_MODE;
    w.config.pid_out_type = Motor::CURRENT_OUTPUT;
    w.config.speed_pid_config = &speed_pid;
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    w.SetEnable(rc.is_enable);
    const UnitFloat<rpm> speed = rc.x * MAX_SPEED;
    w.SetSpeed(speed);
    w.OnLoop();

    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}