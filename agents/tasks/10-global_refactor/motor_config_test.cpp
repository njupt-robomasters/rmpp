// 验证配置默认值、反馈换算、外部校零和超时
#include "motor/motor.hpp"
#include <cassert>
#include <cmath>

static uint64_t now_cycles = 1000000;

float Timer::elapsed() const {
    return static_cast<float>(now_cycles - last_cycles) * 1e-6f;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_cycles;
    return dt;
}

class TestMotor : public Motor {
  public:
    TestMotor(const Config& config)
        : Motor(config, 10, 2, 4) {}

    void feedback() {
        raw_current = 1;
        raw_torque = 0.2f;
        raw_speed = 8;
        raw_position = 0.6f;
        can_callback();
    }
};

static bool near(const float a, const float b) {
    return std::fabs(a - b) < 1e-5f;
}

int main() {
    const Motor::Config input{.can = Can(1), .tx_id = 0x12, .rx_id = 0x13, .timeout = 0.05f, .is_invert = true, .offset = 0.1f};
    TestMotor motor(input);
    assert(input.max_current == 0 && input.max_torque == 0 && input.reduction == 0);
    assert(motor.config.max_current == 10 && motor.config.max_torque == 2 && motor.config.reduction == 4);
    assert(motor.config.tx_id == 0x12 && motor.config.rx_id == 0x13);
    motor.set_current(20);
    assert(motor.current_ref == 10 && motor.torque_ref == 2);
    motor.set_torque(-3);
    assert(motor.current_ref == -10 && motor.torque_ref == -2);
    motor.feedback();
    assert(motor.is_connect && near(motor.current, -1) && near(motor.speed, -2) && near(motor.position, -0.125f));
    motor.config.offset += (motor.config.is_invert ? -motor.position : motor.position) * motor.config.reduction;
    motor.feedback();
    assert(near(motor.position, 0));
    now_cycles += 60000;
    motor.update();
    assert(!motor.is_connect && motor.current == 0 && motor.torque == 0 && motor.speed == 0);
    TestMotor custom({.can = Can(2), .max_current = 20, .max_torque = 5, .reduction = 2});
    custom.set_torque(2.5f);
    assert(custom.config.reduction == 2 && near(custom.current_ref, 10));
}
