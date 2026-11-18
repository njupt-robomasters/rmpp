// 验证配置默认值、可选反馈与电流/力矩换算、外部校零和超时
#include "math/angle.hpp"
#include "motor/motor.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

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

    void feedback(const float angle = 0.6f) {
        can_callback({.current = 1, .speed = 8, .angle = angle});
    }

    void status() {
        can_callback({});
    }

    void torque_feedback(const float torque) {
        can_callback({.torque = torque});
    }
};

static bool near(const float a, const float b) {
    return std::fabs(a - b) < 3e-5f;
}

static void check_multi_turn(const float reduction, const bool is_invert) {
    const float origin = 5.8f;
    const float dir = is_invert ? -1.0f : 1.0f;
    TestMotor motor({.can = Can(1), .reduction = reduction, .is_invert = is_invert, .offset = origin});
    // RMD 电流应答可早于首个角度应答
    motor.status();
    motor.feedback(origin);
    assert(near(motor.angle, 0) && near(motor.angle_multi, 0));

    // 正转跨圈再反转，编码器每次变化小于半圈
    float previous = 0;
    for (int i = 0; i <= 160; i++) {
        const float travel = (i <= 60 ? i : 120 - i) * 0.2f;
        const float raw = origin + travel;
        float wrapped = std::fmod(raw, rev);
        if (wrapped < 0)
            wrapped += rev;
        motor.feedback(wrapped);
        motor.status();
        const float expected = dir * travel / reduction;
        assert(near(motor.angle_multi, expected));
        assert(near(motor.angle, Angle(expected)));
        assert(std::fabs(motor.angle_multi - previous) < 0.21f / reduction);
        previous = motor.angle_multi;
    }

    // 外部校零同时作用于单圈和连续角度；重复反馈不会重复扣 offset
    const float raw = std::fmod(origin - 8 + 2 * rev, rev);
    motor.config.offset += dir * motor.angle_multi * reduction;
    for (int i = 0; i < 3; i++) {
        motor.feedback(raw);
        assert(near(motor.angle, 0) && near(motor.angle_multi, 0));
    }
    motor.config.offset += 0.7f;
    motor.feedback(raw);
    assert(near(motor.angle_multi, -dir * 0.7f / reduction));
    const float last = motor.angle_multi;
    now_cycles += 200000;
    motor.update();
    assert(!motor.is_connect && motor.angle == 0 && motor.angle_multi == last);
    motor.update();
    assert(motor.angle == 0 && motor.angle_multi == last);
    motor.status();
    assert(motor.is_connect && near(motor.angle, Angle(last)) && motor.angle_multi == last);
    motor.feedback(raw + 0.1f);
    assert(near(motor.angle_multi, last + dir * 0.1f / reduction));
    assert(near(motor.angle, Angle(motor.angle_multi)));
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
    assert(motor.is_connect && near(motor.current, -1) && near(motor.torque, -0.2f) && near(motor.speed, -2) &&
           near(motor.angle, -0.125f));
    motor.status();
    assert(near(motor.current, -1) && near(motor.torque, -0.2f) && near(motor.speed, -2) && near(motor.angle, -0.125f));
    motor.config.offset += (motor.config.is_invert ? -motor.angle_multi : motor.angle_multi) * motor.config.reduction;
    motor.feedback();
    assert(near(motor.angle, 0));
    now_cycles += 60000;
    motor.update();
    assert(!motor.is_connect && motor.current == 0 && motor.torque == 0 && motor.speed == 0);
    TestMotor custom({.can = Can(2), .max_current = 20, .max_torque = 5, .reduction = 2});
    custom.set_torque(2.5f);
    assert(custom.config.reduction == 2 && near(custom.current_ref, 10));
    custom.torque_feedback(1);
    assert(near(custom.torque, 1) && near(custom.current, 4) && custom.speed == 0);
    for (const float reduction : {1.0f, 4.0f, 19.2f})
        for (const bool is_invert : {false, true})
            check_multi_turn(reduction, is_invert);
    std::puts("PASS: motor config and multi-turn angle regression");
}
