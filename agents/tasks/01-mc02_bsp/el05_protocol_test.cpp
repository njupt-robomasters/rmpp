// 主机测试：真实 EL05 驱动的私有扩展帧收发、筛帧和停止保护
#include "motor/el05.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

static uint64_t now_us = 1000000;
static CanFrame sent;
static Can::RxCallback receive;

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame& frame) const {
    sent = frame;
}

void Can::add_rx_callback(RxCallback callback) const {
    receive = callback;
}

void Timer::delay(float) {}

float Timer::elapsed() const {
    return static_cast<float>(now_us - last_cycles) * 1e-6f;
}

float Timer::restart() {
    const float dt = elapsed();
    last_cycles = now_us;
    return dt;
}

static void check_frame(const uint32_t id, const uint8_t first = 0) {
    assert(sent.is_extend && !sent.is_remote && sent.size == 8);
    assert(sent.id == id);
    assert(sent.data[0] == first);
    for (int i = 1; i < 8; i++)
        assert(sent.data[i] == 0);
}

static void check_torque(const uint32_t id) {
    assert(sent.is_extend && !sent.is_remote && sent.size == 8);
    assert(sent.id == id);
    const uint8_t data[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0, 0, 0, 0};
    for (int i = 0; i < 8; i++)
        assert(sent.data[i] == data[i]);
}

int main() {
    EL05 motor({.can = Can(2), .tx_id = 0x12, .rx_id = 0x22, .max_current = 5.0f / 0.94f, .max_torque = 5});
    motor.send();
    check_frame(0x04002212); // 失能停止
    motor.set_enable(true);
    motor.set_output(3);
    motor.send();
    check_frame(0x03002212); // 首次使能，主机与电机 ID 在扩展 ID 中
    now_us += 2000;
    motor.send();
    check_torque(0x017FFF12); // 首次反馈前保持零力矩

    const CanFrame feedback = {
        .id = 0x02801222,
        .is_extend = true,
        .size = 8,
        .data = {0x7F, 0xFF, 0xBF, 0xFF, 0x3F, 0xFF, 0x01, 0x2C},
    };
    for (int i = 0; i < 6; i++) {
        CanFrame invalid = feedback;
        if (i == 0)
            invalid.is_extend = false;
        else if (i == 1)
            invalid.is_remote = true;
        else if (i == 2)
            invalid.size = 7;
        else if (i == 3)
            invalid.id = 0x02801322; // 其他电机
        else if (i == 4)
            invalid.id = 0x02801223; // 其他主机
        else
            invalid.id = 0x01801222; // 其他通信类型
        receive(invalid);
        assert(!motor.is_connect);
    }
    // 同总线标准帧即使 ID/首字节碰巧符合旧 MIT 筛选条件，也须拒绝
    receive({.id = 0x22, .size = 8, .data = {0x12, 0, 0, 0, 0, 0, 0, 0}});
    assert(!motor.is_connect);

    receive(feedback);
    assert(motor.is_connect && motor.mode == EL05::MOTOR_MODE && motor.fault == 0);
    assert(std::fabs(motor.position) < 0.001f);
    assert(std::fabs(motor.speed - 25) < 0.002f);
    assert(std::fabs(motor.torque + 3) < 0.001f);
    assert(std::fabs(motor.current + 3 / 0.94f) < 0.001f);
    assert(motor.temperature == 30);
    motor.send();
    check_torque(0x01BFFF12); // +3 Nm 编码在 ID，p/v/kp/kd 为零
    motor.set_output(-3);
    motor.send();
    check_torque(0x013FFF12);
    motor.set_output(100);
    assert(motor.torque_ref == 5);
    motor.send();
    check_torque(0x01EAA912); // 保留应用 5 Nm 限幅
    motor.set_output(std::numeric_limits<float>::quiet_NaN());
    motor.send();
    check_torque(0x017FFF12);
    motor.set_output(3);

    CanFrame reset = feedback;
    reset.id = 0x02001222;
    receive(reset);
    motor.send();
    check_torque(0x017FFF12); // 非运行状态保持零力矩
    receive(feedback);
    now_us += 100000;
    motor.send();
    check_frame(0x03002212); // 按时间刷新使能
    now_us += 2000;
    motor.update();
    assert(!motor.is_connect);
    motor.send();
    check_torque(0x017FFF12); // 反馈超时停止出力

    CanFrame fault = feedback;
    fault.id = 0x02BF1222;
    receive(fault);
    assert(motor.fault == 0x3F && motor.mode == EL05::MOTOR_MODE);
    motor.send();
    check_frame(0x04002212);
    now_us += 499000;
    motor.send();
    check_frame(0x04002212);
    now_us += 2000;
    motor.send();
    check_frame(0x04002212, 1); // 私有协议清错：停止帧 Byte0=1
    assert(motor.clear_fault_count == 1);
    receive(feedback);
    motor.send();
    check_frame(0x03002212); // 无故障后重新使能
    motor.set_enable(false);
    motor.send();
    check_frame(0x04002212);

    EL05 inverted({.can = Can(2), .tx_id = 0x12, .rx_id = 0x22, .is_invert = true});
    receive(feedback);
    assert(inverted.speed < 0 && inverted.torque > 0);
    inverted.set_enable(true);
    inverted.set_output(3);
    inverted.send();
    check_frame(0x03002212);
    inverted.send();
    check_torque(0x013FFF12);
    std::puts("EL05 private CAN protocol checks passed");
}
