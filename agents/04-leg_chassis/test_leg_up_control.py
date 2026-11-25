"""用轻量设备替身运行实际上板控制与组帧代码。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/04-leg_chassis/leg_up_control"

CAN_STUB = r"""
#pragma once
#include <cstdint>
#include <functional>
struct CanFrame {
    uint32_t id = 0;
    unsigned size = 0;
    uint8_t data[8]{};
};
// 回调与发送帧共享，便于测试注入状态帧、读取控制帧
struct Can {
    explicit Can(int) {}
    void add_rx_callback(const std::function<void(const CanFrame&)>& callback) const { this->rx = callback; }
    void send(const CanFrame&) const {}
    inline static std::function<void(const CanFrame&)> rx;
};
"""

TIMER_STUB = r"""
#pragma once
struct Timer {
    inline static float now = 0;
    float start = 0;
    float elapsed() const { return now - start; }
    void restart() { start = now; }
};
"""

STUBS = r"""
#include <cassert>
#include <cmath>
#include "leg_protocol.hpp"

constexpr float deg_s = 3.14159265358979323846f / 180;
struct VT13 {
    struct {
        bool fn = false, pause = false;
    } rc;
    struct { bool e = false, z = false, x = false, c = false; } key;
    bool is_enable = false, is_aim = false;
    float vx = 0, wr = 0, yaw_speed = 0;
} vt13;
struct {
    bool is_detect = false;
} aim;
struct { bool is_connect = true; float angle = 0; } yaw;
struct {
    struct { float chassis_power_limit = 120; } robot_status;
    struct { float buffer_energy = 60; } power_heat_data;
} referee;
struct SuperCap {
    enum Mode { OFF, BOOST } mode = OFF;
    unsigned updates = 0;
    float power_chassis_ref = 87.3f;
    void set_mode(Mode value) { mode = value; }
    void set_referee_limit(float) {}
    void set_referee_energy(float) {}
    void update() { updates++; }
} super_cap;
LegUpProtocol leg_protocol(Can(1));
"""

TESTS = r"""
void report(const bool is_fallen, const bool is_enable = false) {
    CanFrame frame;
    LegDownFrame{.is_fallen = is_fallen, .is_enable = is_enable}.pack(frame);
    Can::rx(frame);
}
CanFrame cycle() {
    chassis_loop();
    CanFrame frame;
    leg_protocol.pack(frame);
    return frame;
}
LegUpFrame command() {
    LegUpFrame result;
    assert(result.unpack(cycle()));
    return result;
}
void press_pause() {
    vt13.rc.pause = false;
    cycle();
    vt13.rc.pause = true;
    cycle();
}
uint64_t field(const CanFrame& frame, const unsigned offset, const unsigned bits) {
    uint64_t value = 0;
    std::memcpy(&value, frame.data, frame.size);
    return value >> offset & ((uint64_t(1) << bits) - 1);
}
int main() {
    // 首次状态帧到达前保持失能与低腿长
    vt13.is_enable = true;
    assert(!command().is_enable && leg_protocol.leg_length == 0.14f);

    // 下板在线后仍需首次 pause 才给使能
    report(true);
    auto frame = cycle();
    assert(frame.id == UP_ID && frame.size == 8);
    assert(!leg_protocol.is_enable && leg_protocol.leg_length == 0.14f);
    assert(super_cap.updates == 2 && super_cap.mode == SuperCap::BOOST);
    press_pause();
    assert(leg_protocol.is_enable && leg_protocol.leg_length == 0.14f);

    // 之后 pause 在低、中、高之间往返
    press_pause();
    assert(leg_protocol.leg_length == 0.22f);
    press_pause();
    assert(leg_protocol.leg_length == 0.33f);
    press_pause();
    assert(leg_protocol.leg_length == 0.22f);
    press_pause();
    assert(leg_protocol.leg_length == 0.14f);

    vt13.key.c = true;
    assert(command().leg_length == 0.33f);
    press_pause();
    assert(command().leg_length == 0.22f);
    vt13.key.z = true;
    assert(command().leg_length == 0.14f);
    vt13.key.x = true;
    assert(command().leg_length == 0.22f);
    vt13.key = {};
    cycle();

    // E 或 Fn 按住期间持续下发跳跃位
    vt13.key.e = true;
    assert(command().trigger_jump);
    vt13.rc.fn = true;
    assert(command().trigger_jump);
    vt13.key.e = false;
    assert(command().trigger_jump);
    vt13.rc.fn = false;
    assert(!command().trigger_jump);

    // 运动指令与 yaw 离线
    vt13.vx = -1.5f;
    vt13.wr = -1.0f;
    yaw.angle = -0.3f;
    auto decoded = command();
    assert(std::fabs(decoded.vx + 1.5f * std::cos(yaw.angle)) <= 0.0005f);
    assert(std::fabs(decoded.wr + 1.0f) <= 0.0005f);
    assert(decoded.yaw == 0);
    vt13.wr = 0;
    vt13.yaw_speed = 0.25f;
    decoded = command();
    assert(decoded.wr == 0.25f);
    assert(decoded.yaw == -0.3f);
    vt13.is_aim = true;
    assert(command().wr == 0.25f);
    aim.is_detect = true;
    decoded = command();
    assert(decoded.wr == 0 && decoded.yaw == -0.3f);
    vt13.is_aim = false;
    assert(command().wr == 0.25f);
    yaw.is_connect = false;
    yaw.angle = 0; // Motor::update() 在实际云台循环中清零离线角度
    assert(command().yaw == 0);

    // 失能清使能，重新使能需再按 pause
    vt13.is_enable = false;
    vt13.rc.fn = true;
    assert(!command().is_enable && leg_protocol.trigger_jump);
    assert(super_cap.mode == SuperCap::OFF);
    vt13.is_enable = true;
    assert(!command().is_enable && leg_protocol.trigger_jump);
    assert(super_cap.mode == SuperCap::BOOST);
    cycle();
    vt13.rc.fn = false;
    press_pause();
    assert(command().is_enable && !leg_protocol.trigger_jump);

    // 下板断联清反馈与使能、复位腿长
    leg_protocol.leg_length = 0.33f;
    Timer::now = 0.2f;
    leg_protocol.received.wr = 1;
    leg_protocol.received.is_enable = true;
    cycle();
    assert(!leg_protocol.is_connect && leg_protocol.received.wr == 0 && !leg_protocol.received.is_fallen && !leg_protocol.received.is_enable);
    assert(!leg_protocol.is_enable && leg_protocol.leg_length == 0.14f);

    // 重新连接后需再按 pause 使能
    report(false);
    assert(!command().is_enable && leg_protocol.leg_length == 0.14f);
    press_pause();
    assert(command().is_enable && leg_protocol.leg_length == 0.14f);
    press_pause();
    assert(command().leg_length == 0.22f);

    // 非法帧保持原数据
    report(true, true);
    assert(leg_protocol.received.is_fallen && leg_protocol.received.is_enable);
    chassis_loop(); // 未超时，数据保留
    assert(leg_protocol.received.is_fallen && leg_protocol.received.is_enable);

    // 上板帧位布局：使能 0、跳跃 1、vx 2..14、wr 15..30、yaw 31..43、腿长 44..52、功率 53..63
    const LegUpFrame layout{
        .is_enable = true,
        .trigger_jump = true,
        .vx = -1.234f,
        .wr = 2.345f,
        .yaw = -3.0f,
        .leg_length = 0.257f,
        .power_ref = 80.0f,
    };
    layout.pack(frame);
    assert(field(frame, 0, 1) == 1 && field(frame, 1, 1) == 1);
    assert(field(frame, 2, 13) == ((1u << 13) - 1234));
    assert(field(frame, 15, 16) == 2345);
    assert(field(frame, 31, 13) == ((1u << 13) - 3000));
    assert(field(frame, 44, 9) == 257);
    assert(field(frame, 53, 11) == 320);

    // 下板帧位布局：倒地 0、使能 1、wr 2..17、6 个电机 18..29、保留 30..31
    LegDownFrame status;
    status.is_fallen = true;
    status.is_enable = true;
    status.wr = -1.234f;
    status.motor[0] = LegDownFrame::MotorState::NORMAL;   // 左前
    status.motor[1] = LegDownFrame::MotorState::OVERHEAT; // 左后
    status.motor[2] = LegDownFrame::MotorState::ERROR;    // 右后
    status.motor[3] = LegDownFrame::MotorState::OFFLINE;  // 右前
    status.motor[4] = LegDownFrame::MotorState::ERROR;    // 左轮
    status.motor[5] = LegDownFrame::MotorState::NORMAL;   // 右轮
    status.pack(frame);
    assert(frame.size == 4);
    assert(field(frame, 0, 1) == 1 && field(frame, 1, 1) == 1);
    assert(field(frame, 2, 16) == ((1u << 16) - 1234));
    assert(field(frame, 18, 2) == 0 && field(frame, 20, 2) == 1);
    assert(field(frame, 22, 2) == 2 && field(frame, 24, 2) == 3);
    assert(field(frame, 26, 2) == 2 && field(frame, 28, 2) == 0);
    assert(field(frame, 30, 2) == 0);

    // 下板状态帧编解码与接收
    LegDownFrame feedback;
    assert(feedback.unpack(frame));
    assert(std::fabs(feedback.wr + 1.234f) < 0.00001f && feedback.is_fallen && feedback.is_enable);
    assert(feedback.motor[1] == LegDownFrame::MotorState::OVERHEAT && feedback.motor[3] == LegDownFrame::MotorState::OFFLINE);
    Can::rx(frame);
    assert(std::fabs(leg_protocol.received.wr + 1.234f) < 0.00001f && leg_protocol.received.is_fallen && leg_protocol.received.is_enable);
    assert(leg_protocol.received.motor[2] == LegDownFrame::MotorState::ERROR && leg_protocol.received.motor[5] == LegDownFrame::MotorState::NORMAL);
    frame.id = UP_ID;
    assert(!feedback.unpack(frame));
    frame.id = DOWN_ID;
    frame.size = 3;
    assert(!feedback.unpack(frame));
    assert(feedback.is_fallen && feedback.is_enable);

    // 腿长编解码
    LegUpFrame received_up;
    for (unsigned length = 0; length <= 511; length++) {
        leg_protocol.leg_length = length / 1000.0f;
        frame = cycle();
        assert(received_up.unpack(frame));
        assert(received_up.leg_length == length / 1000.0f);
        assert(received_up.is_enable);
    }

    // 错误 ID 或长度保持原数据
    frame.id = DOWN_ID;
    assert(!received_up.unpack(frame));
    frame.id = UP_ID;
    frame.size = 7;
    assert(!received_up.unpack(frame));
    assert(received_up.leg_length == 0.511f && received_up.is_enable);

    // 速度、角度、腿长限幅
    for (const float speed : {-40.0f, -32.768f, -4.096f, -3.0f, -0.001f, 0.0f, 0.001f, 3.0f, 4.095f, 32.767f, 40.0f}) {
        const LegUpFrame input{.vx = speed, .wr = speed, .yaw = -3.1415f, .leg_length = 0.257f};
        input.pack(frame);
        assert(decoded.unpack(frame));
        assert(std::fabs(decoded.vx - std::clamp(speed, -4.096f, 4.095f)) <= 0.0005f);
        assert(std::fabs(decoded.wr - std::clamp(speed, -32.768f, 32.767f)) <= 0.0005f);
        assert(std::fabs(decoded.yaw + 3.1415f) <= 0.0005f && decoded.leg_length == 0.257f);
    }
    for (const float length : {-1.0f, 0.1424f, 0.1426f, 1.0f}) {
        const LegUpFrame input{.leg_length = length};
        input.pack(frame);
        assert(decoded.unpack(frame));
        assert(std::fabs(decoded.leg_length - std::clamp(length, 0.0f, 0.511f)) <= 0.000501f);
    }

    // 功率目标：0.25 W/LSB，有限值限幅到 0..511.5 W，INFINITY 编码为 2047；上板取超电 power_chassis_ref
    for (const float power : {-10.0f, 0.0f, 0.1f, 80.0f, 87.3f, 511.5f, 600.0f}) {
        const LegUpFrame input{.power_ref = power};
        input.pack(frame);
        assert(decoded.unpack(frame));
        assert(std::fabs(decoded.power_ref - std::clamp(power, 0.0f, 511.5f)) <= 0.125f);
    }
    {
        const LegUpFrame input{.power_ref = INFINITY};
        input.pack(frame);
        assert(field(frame, 53, 11) == 2047);
        assert(decoded.unpack(frame) && std::isinf(decoded.power_ref));
    }
    frame = cycle();
    assert(received_up.unpack(frame) && received_up.power_ref == 87.25f);
}
"""


def main():
    up = ROOT / "app/leg/leg_up"
    down = ROOT / "app/leg/leg_down"
    assert (ROOT / "app/leg/leg_protocol.hpp").exists()
    assert not (up / "leg_protocol.hpp").exists()
    assert not (down / "leg_protocol.hpp").exists()
    assert '#include "../leg_protocol.hpp"' in (up / "chassis.hpp").read_text(encoding="utf-8")
    assert '#include "../leg_protocol.hpp"' in (down / "chassis.hpp").read_text(encoding="utf-8")
    for path in down.glob("*.cpp"):
        assert "Can(2)" not in path.read_text(encoding="utf-8")
    assert "Can(2)" not in (down / "chassis.hpp").read_text(encoding="utf-8")
    assert "SuperCap" not in (down / "chassis.hpp").read_text(encoding="utf-8")
    assert "super_cap" not in (down / "app.cpp").read_text(encoding="utf-8")
    assert "Mode mode = OFF;" in (ROOT / "lib/power/super_cap.hpp").read_text(encoding="utf-8")
    source = (up / "chassis.cpp").read_text(encoding="utf-8")
    source = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
    constants = "\n".join(line for line in (up / "chassis.hpp").read_text(encoding="utf-8").splitlines()
                          if line.startswith("inline constexpr float LEG_LENGTH"))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "bsp").mkdir(exist_ok=True)
    (OUT / "bsp/can.hpp").write_text(CAN_STUB, encoding="utf-8")
    (OUT / "bsp/timer.hpp").write_text(TIMER_STUB, encoding="utf-8")
    generated = OUT / "test.cpp"
    generated.write_text(STUBS + constants + "\n" + source + TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-I", str(OUT), "-I", str(ROOT / "app/leg"),
                    str(generated), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    print("PASS: shared protocol, CAN1 wiring, control state transitions, bit layout and frame encoding")


if __name__ == "__main__":
    main()
