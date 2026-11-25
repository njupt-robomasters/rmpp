// 主机测试：原生输出分派、连续卡弹检测、退弹时长与停止优先级
#include "motor/dm4310.hpp"
#include "motor/vesc.hpp"
#include "motor/x630.hpp"
#include "robot/shooter_17mm.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

static uint64_t now_us = 1000000;
static CanFrame sent;

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame& frame) const {
    sent = frame;
}

void Can::add_rx_callback(RxCallback) const {}

void Timer::delay(float) {}

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    const float dt = elapsed();
    last_cycles = now_us;
    return dt;
}

class TestMotor : public Motor {
  public:
    TestMotor()
        : Motor({.can = Can(1)}, 20, 6, 1) {}

    void feedback(const float speed = 0) {
        can_callback({.speed = speed});
    }
};

int main() {
    Vesc rub({.can = Can(1), .tx_id = 1, .max_current = 20, .max_torque = 6, .reduction = 1});
    TestMotor motor;
    const Pid::Config pid = {.kp = 20, .max_out = 20};
    Shooter17mm shooter({rub, rub, motor}, {.shoot_pid_config = &pid, .bullet_per_angle = 1, .block_time = 0.0045f, .reverse_time = 0.0025f});
    const auto tick = [&](const float speed = 0, const uint64_t step_us = 1000, const bool online = true) {
        now_us += step_us;
        if (online)
            motor.feedback(speed);
        shooter.update();
        return motor.current_ref;
    };
    shooter.set_enable(true);
    shooter.set_bullet_freq(2);
    assert(tick() > 0); // 摩擦轮关闭、离线时仍可独立拨弹
    shooter.set_bullet_freq(0);
    assert(tick() == 0);
    for (int i = 0; i < 10; i++) {
        assert(tick(1) < 0); // 零速度闭环持续制动，饱和也不触发退弹
        assert(tick(-1) > 0);
    }
    assert(tick() == 0);
    shooter.set_bullet_freq(2);

    // 未饱和的一帧打断连续卡弹计时
    for (int i = 0; i < 4; i++)
        assert(tick() > 0);
    assert(tick(2) == 0);
    // 上一周期的零电流在本周期清除计时，随后累计五个饱和周期
    for (int i = 0; i < 6; i++)
        assert(tick() > 0);
    for (int i = 0; i < 3; i++)
        assert(tick() < 0);
    assert(tick() > 0);

    // 退弹时热量保护立即进入零速度闭环，恢复后重新计时
    shooter.set_heat_remain(-1);
    assert(tick(1) < 0);
    shooter.set_heat_remain(100);
    for (int i = 0; i < 5; i++)
        assert(tick() > 0);
    assert(tick() < 0);
    shooter.set_heat_remain(-1);
    assert(tick() == 0);
    shooter.set_heat_remain(100);

    // 负弹频按负方向拨弹，卡弹时向正方向退弹
    shooter.set_bullet_freq(-2);
    for (int i = 0; i < 6; i++)
        assert(tick() < 0);
    assert(tick() > 0);
    shooter.set_bullet_freq(0);
    assert(tick() == 0);
    shooter.set_bullet_freq(2);
    assert(tick() > 0);
    shooter.set_enable(false);
    assert(tick(1) == 0);
    assert(tick(-1) == 0);
    shooter.set_enable(true);
    assert(tick() > 0);
    assert(tick(1) > 0);
    assert(tick(0, 200000, false) == 0); // 离线时电机层清零速度反馈，零速度闭环输出为零
    assert(tick() > 0);

    // 长停顿最多累计 DT_MAX，保持前进直到后续控制周期累计到阈值
    Shooter17mm slow({rub, rub, motor}, {.shoot_pid_config = &pid, .bullet_per_angle = 1, .block_time = 0.0245f});
    slow.set_enable(true);
    slow.set_bullet_freq(2);
    now_us += 1000000;
    motor.feedback();
    slow.update();
    for (int i = 0; i < 5; i++) {
        now_us += 1000;
        motor.feedback();
        slow.update();
        assert(motor.current_ref > 0);
    }

    // PID 输出饱和即计卡弹：max_out 低于电机限幅时，按 PID 限幅检测并按配置反转
    const Pid::Config saturate_pid = {.kp = 5, .max_out = 5};
    Shooter17mm saturate({rub, rub, motor}, {.shoot_pid_config = &saturate_pid, .bullet_per_angle = 1, .block_time = 0.0025f, .reverse_time = 0.0015f});
    saturate.set_enable(true);
    saturate.set_bullet_freq(2);
    for (int i = 0; i < 7; i++) {
        now_us += 1000;
        motor.feedback();
        saturate.update();
        assert(std::fabs(motor.current_ref) == 5);
        assert((motor.current_ref > 0) == (i < 4 || i == 6));
    }

    // 堵转时 P 输出低于 max_out 则不饱和，持续前进
    saturate.set_bullet_freq(0.5f);
    for (int i = 0; i < 4; i++) {
        now_us += 1000;
        motor.feedback();
        saturate.update();
    }
    for (int i = 0; i < 10; i++) {
        now_us += 1000;
        motor.feedback();
        saturate.update();
        assert(motor.current_ref == 2.5f);
    }

    // 基类引用分派：默认电流与 DM 力矩
    Motor& current_motor = motor;
    current_motor.set_output(2);
    assert(motor.current_ref == 2);
    DM4310 dm({.can = Can(1), .tx_id = 0x12, .rx_id = 0x13});
    Motor& torque_motor = dm;
    torque_motor.set_output(2);
    assert(dm.torque_ref == 2);

    // VESC 仍生成电流命令，X630 仍执行静音逻辑
    rub.set_enable(true);
    rub.is_connect = true;
    Motor& vesc_motor = rub;
    vesc_motor.set_output(2);
    rub.send();
    assert(sent.id == (1u << 8 | 1u));
    assert(sent.data[2] == 0x07 && sent.data[3] == 0xd0);
    X630 x630({.can = Can(1), .rx_id = 0x201}, {});
    Motor& mute_motor = x630;
    mute_motor.set_output(0.5f);
    assert(x630.current_ref == 0);
    mute_motor.set_output(3);
    assert(x630.current_ref == 3);
    std::puts("PASS: native output dispatch, feed/reverse timing, stop/reset, dt clamp");
}
