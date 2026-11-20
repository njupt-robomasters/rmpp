"""用轻量设备替身运行实际上板控制与组帧代码。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "out" / "leg_up_control"

CAN_STUB = r"""
#pragma once
#include <cstdint>
struct CanFrame {
    uint32_t id = 0;
    unsigned size = 0;
    uint8_t data[8]{};
};
"""

STUBS = r"""
#include <cassert>
#include "leg_protocol.hpp"

constexpr float deg_s = 3.14159265358979323846f / 180;
struct Timer {
    inline static float now = 0;
    float start = 0;
    float elapsed() const { return now - start; }
    void restart() { start = now; }
};
struct VT13 {
    struct {
        bool fn = false, pause = false;
    } rc;
    bool is_enable = false;
    float vx = 0, wr = 0;
} vt13;
struct { bool is_connect = true; float angle = 0; } yaw;
LegDownFrame down;
struct SuperCap {
    enum Mode { OFF };
    float power_chassis_ref = 132;
    unsigned updates = 0;
    void set_mode(Mode) {}
    void update() { updates++; }
} super_cap;
"""

DOWN_STUBS = r"""
LegUpFrame received_up;
bool up_is_connect = false;
Timer down_callback_timer;
"""

TESTS = r"""
void report(const bool is_fallen) {
    CanFrame frame;
    LegDownFrame{.is_fallen = is_fallen}.pack(frame);
    chassis_can_callback(frame);
}
CanFrame cycle() {
    chassis_loop();
    CanFrame frame;
    chassis_write_can_frame(frame);
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
    std::memcpy(&value, frame.data, sizeof(value));
    return value >> offset & ((uint64_t(1) << bits) - 1);
}
int main() {
    // 首次状态帧到达前保持失能与低腿长
    vt13.is_enable = true;
    assert(!command().is_enable && up.leg_length == 0.14f);
    // 倒地使能：pause 授权自救并保持，腿长不变
    report(true);
    auto frame = cycle();
    assert(frame.id == UP_ID && frame.size == 8);
    assert(up.is_enable && !up.allow_rescue && up.leg_length == 0.14f);
    assert(super_cap.updates == 2);
    press_pause();
    assert(up.allow_rescue && up.leg_length == 0.14f);
    press_pause();
    assert(command().leg_length == 0.14f && up.allow_rescue);

    // 站起后 pause 切换腿长，授权保持
    report(false);
    assert(command().leg_length == 0.14f);
    press_pause();
    assert(command().leg_length == 0.22f);
    assert(command().leg_length == 0.22f);
    press_pause();
    assert(command().leg_length == 0.33f);
    press_pause();
    assert(command().leg_length == 0.14f && up.allow_rescue);

    // 跳跃上升沿
    vt13.rc.fn = true;
    assert(command().trigger_jump);
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
    assert(decoded.yaw == -0.3f);
    yaw.is_connect = false;
    yaw.angle = 0; // Motor::update() 在实际云台循环中清零离线角度
    assert(command().yaw == 0);

    // 失能清授权，fn 保持直接透传
    vt13.is_enable = false;
    vt13.rc.fn = true;
    assert(!command().is_enable && !up.allow_rescue && up.trigger_jump);
    vt13.is_enable = true;
    assert(command().is_enable && up.trigger_jump && !up.allow_rescue);
    assert(command().trigger_jump);
    vt13.rc.fn = false;
    cycle();
    vt13.rc.fn = true;
    assert(command().trigger_jump && !up.allow_rescue);

    // 下板断联清反馈与授权、复位腿长
    report(true);
    press_pause();
    assert(up.allow_rescue);
    up.leg_length = 0.33f;
    Timer::now = 0.2f;
    down.wr = 1;
    cycle();
    assert(!down_is_connect && down.wr == 0 && !down.is_fallen);
    assert(!up.is_enable && !up.allow_rescue && up.leg_length == 0.14f);

    // 站立使能时直接切换腿长
    report(false);
    assert(command().is_enable && !up.allow_rescue && up.leg_length == 0.14f);
    press_pause();
    assert(command().leg_length == 0.22f && !up.allow_rescue);
    report(true);
    press_pause();
    assert(command().allow_rescue && up.leg_length == 0.22f);
    CanFrame invalid{.id = DOWN_ID, .size = 2};
    chassis_can_callback(invalid);
    assert(down.is_fallen);

    // 上板帧位布局：开关 0..2、vx 3..15、wr 16..30、yaw 31..43、腿长 44..52、功率 53..63
    const LegUpFrame layout{
        .is_enable = true,
        .allow_rescue = false,
        .trigger_jump = true,
        .vx = -1.234f,
        .wr = 2.345f,
        .yaw = -3.0f,
        .leg_length = 0.257f,
        .power_ref = 100.25f,
    };
    layout.pack(frame);
    assert(field(frame, 0, 1) == 1 && field(frame, 1, 1) == 0 && field(frame, 2, 1) == 1);
    assert(field(frame, 3, 13) == ((1u << 13) - 1234));
    assert(field(frame, 16, 15) == 2345);
    assert(field(frame, 31, 13) == ((1u << 13) - 3000));
    assert(field(frame, 44, 9) == 257);
    assert(field(frame, 53, 11) == 401);

    // 下板帧位布局：倒地 0、wr 1..16、保留 17..23
    LegDownFrame{.is_fallen = true, .wr = -1.234f}.pack(frame);
    assert(frame.size == 3 && (frame.data[0] & 1) == 1);
    assert((((frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16) >> 1) & 0xFFFF) == 0x10000 - 1234);
    assert((frame.data[2] >> 1) == 0);

    // 腿长与功率编解码
    for (unsigned length = 0; length <= 511; length++) {
        up.leg_length = length / 1000.0f;
        super_cap.power_chassis_ref = 133;
        frame = cycle();
        down_callback(frame);
        assert(up_is_connect && received_up.power_ref == 133);
        assert(received_up.leg_length == length / 1000.0f);
        assert(received_up.is_enable && received_up.allow_rescue);
    }
    super_cap.power_chassis_ref = INFINITY;
    frame = cycle();
    assert(field(frame, 53, 11) == 2047);
    down_callback(frame);
    assert(std::isinf(received_up.power_ref));
    super_cap.power_chassis_ref = 0;
    frame = cycle();
    down_callback(frame);
    assert(received_up.power_ref == 0);
    super_cap.power_chassis_ref = 2000;
    frame = cycle();
    assert(field(frame, 53, 11) == 2046);
    down_callback(frame);
    assert(received_up.power_ref == 511.5f);

    // 错误 ID 或长度保持原数据
    frame.id = DOWN_ID;
    assert(!received_up.unpack(frame));
    frame.id = UP_ID;
    frame.size = 7;
    assert(!received_up.unpack(frame));
    assert(received_up.power_ref == 511.5f && received_up.is_enable);

    // 速度、角度、腿长限幅
    for (const float speed : {-20.0f, -16.384f, -4.096f, -3.0f, -0.001f, 0.0f, 0.001f, 3.0f, 4.095f, 16.383f, 20.0f}) {
        const LegUpFrame input{.vx = speed, .wr = speed, .yaw = -3.1415f, .leg_length = 0.257f};
        input.pack(frame);
        assert(decoded.unpack(frame));
        assert(std::fabs(decoded.vx - std::clamp(speed, -4.096f, 4.095f)) <= 0.0005f);
        assert(std::fabs(decoded.wr - std::clamp(speed, -16.384f, 16.383f)) <= 0.0005f);
        assert(std::fabs(decoded.yaw + 3.1415f) <= 0.0005f && decoded.leg_length == 0.257f);
    }
    for (const float length : {-1.0f, 0.1424f, 0.1426f, 1.0f}) {
        const LegUpFrame input{.leg_length = length};
        input.pack(frame);
        assert(decoded.unpack(frame));
        assert(std::fabs(decoded.leg_length - std::clamp(length, 0.0f, 0.511f)) <= 0.000501f);
    }

    // 下板状态帧编解码
    const LegDownFrame status{.is_fallen = true, .wr = -1.234f};
    status.pack(frame);
    LegDownFrame feedback;
    assert(feedback.unpack(frame));
    assert(std::fabs(feedback.wr + 1.234f) < 0.00001f && feedback.is_fallen);
    chassis_can_callback(frame);
    assert(std::fabs(down.wr + 1.234f) < 0.00001f && down.is_fallen);
    frame.id = UP_ID;
    assert(!feedback.unpack(frame));
    frame.id = DOWN_ID;
    frame.size = 2;
    assert(!feedback.unpack(frame));
    assert(feedback.is_fallen);
}
"""


def main():
    up = ROOT / "app/leg/leg_up"
    down = ROOT / "app/leg/leg_down"
    assert (ROOT / "app/leg/leg_protocol.hpp").exists()
    assert not (up / "leg_protocol.hpp").exists()
    assert not (down / "leg_protocol.hpp").exists()
    assert '#include "../leg_protocol.hpp"' in (up / "chassis.hpp").read_text(encoding="utf-8")
    assert '#include "../leg_protocol.hpp"' in (down / "chassis.cpp").read_text(encoding="utf-8")
    for path in down.glob("*.cpp"):
        assert "Can(2)" not in path.read_text(encoding="utf-8")
    assert "Can(2)" not in (down / "chassis.hpp").read_text(encoding="utf-8")
    assert "SuperCap" not in (down / "chassis.hpp").read_text(encoding="utf-8")
    assert "super_cap" not in (down / "app.cpp").read_text(encoding="utf-8")
    assert "Mode mode = OFF;" in (ROOT / "lib/power/super_cap.hpp").read_text(encoding="utf-8")
    source = (up / "chassis.cpp").read_text(encoding="utf-8")
    source = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
    constants = "\n".join(line for line in (up / "chassis.hpp").read_text(encoding="utf-8").splitlines()
                          if line.startswith("static constexpr"))
    down_source = (down / "chassis.cpp").read_text(encoding="utf-8")
    decoder = down_source[down_source.index("void chassis_can_callback("):]
    decoder = decoder.replace("chassis_can_callback", "down_callback")
    decoder = decoder.replace("callback_timer", "down_callback_timer")
    decoder = decoder.replace("up.unpack", "received_up.unpack")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "bsp").mkdir(exist_ok=True)
    (OUT / "bsp/can.hpp").write_text(CAN_STUB, encoding="utf-8")
    generated = OUT / "test.cpp"
    generated.write_text(STUBS + constants + "\n" + source + DOWN_STUBS + decoder + TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-I", str(OUT), "-I", str(ROOT / "app/leg"),
                    str(generated), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    print("PASS: shared protocol, CAN1 wiring, control state transitions, bit layout and frame encoding")


if __name__ == "__main__":
    main()
