"""VT13 操控量主机测试：用 Timer、Uart 替身编译实际 lib/rc/vt13.cpp 与 CRC，覆盖帧解析、键盘斜坡、云台、发射、断联与 0x0311 回调。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "out" / "vt13"

STUBS = r"""
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"

uint64_t now_us = 0;
Uart::RxCallback uart_callback;

float Timer::elapsed() const { return static_cast<float>(now_us - last_cycles) * 1e-6f; }
float Timer::restart() {
    const float ret = elapsed();
    last_cycles = now_us;
    return ret;
}
void Uart::add_rx_callback(RxCallback callback) const { uart_callback = callback; }
"""

TESTS = r"""
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "rc/vt13.hpp"
#include "referee/crc.hpp"

extern uint64_t now_us;
extern Uart::RxCallback uart_callback;

static bool near(const float a, const float b) { return std::fabs(a - b) < 1e-4f; }

// 按协议位序写入小端位域
static void put(uint8_t frame[], unsigned& offset, const uint32_t value, const unsigned bits) {
    for (unsigned i = 0; i < bits; i++, offset++)
        if (value >> i & 1u)
            frame[offset / 8] |= static_cast<uint8_t>(1u << offset % 8);
}

struct Input {
    uint16_t ch[4] = {1024, 1024, 1024, 1024}, wheel = 1024;
    uint8_t mode = 1;
    bool pause = false, fn = false, photo = false, trigger = false;
    int16_t mouse_x = 0, mouse_y = 0;
    bool left = false, right = false;
    uint16_t key = 0;
};

static void build(const Input& in, uint8_t frame[21]) {
    std::memset(frame, 0, 21);
    frame[0] = 0xA9;
    frame[1] = 0x53;
    unsigned offset = 16;
    for (const uint16_t ch : in.ch)
        put(frame, offset, ch, 11);
    put(frame, offset, in.mode, 2);
    put(frame, offset, in.pause, 1);
    put(frame, offset, in.fn, 1);
    put(frame, offset, in.photo, 1);
    put(frame, offset, in.wheel, 11);
    put(frame, offset, in.trigger, 1);
    put(frame, offset, 0, 3);
    put(frame, offset, static_cast<uint16_t>(in.mouse_x), 16);
    put(frame, offset, static_cast<uint16_t>(in.mouse_y), 16);
    put(frame, offset, 0, 16);
    put(frame, offset, in.left, 2);
    put(frame, offset, in.right, 2);
    put(frame, offset, 0, 4);
    put(frame, offset, in.key, 16);
    const uint16_t crc = CRC16::check_sum(frame, 19);
    frame[19] = crc & 0xFF;
    frame[20] = crc >> 8;
}

static void send(const Input& in) {
    uint8_t frame[21];
    build(in, frame);
    uart_callback(frame, sizeof(frame));
}

// 追加一帧裁判协议帧：0xA5 帧头、CRC8、命令码、载荷、CRC16
static void append_referee(std::vector<uint8_t>& stream, const uint16_t cmd_id, const uint8_t* data, const size_t size) {
    std::vector<uint8_t> frame{0xA5, static_cast<uint8_t>(size), static_cast<uint8_t>(size >> 8), 0};
    frame.push_back(CRC8::check_sum(frame.data(), 4));
    frame.push_back(static_cast<uint8_t>(cmd_id));
    frame.push_back(static_cast<uint8_t>(cmd_id >> 8));
    frame.insert(frame.end(), data, data + size);
    const uint16_t crc = CRC16::check_sum(frame.data(), static_cast<uint32_t>(frame.size()));
    frame.push_back(static_cast<uint8_t>(crc));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    stream.insert(stream.end(), frame.begin(), frame.end());
}

int main() {
    VT13 vt13({
        .uart = Uart(6),
        .vxy_max = 2.0f,
        .axy = 10.0f,
        .dxy = 5.0f,
        .wr_max = 4.0f,
        .ar = 20.0f,
        .dr = 40.0f,
        .yaw_speed_max = 6.0f,
        .pitch_speed_max = 3.0f,
        .bullet_speed = 25.0f,
        .bullet_freq = 10.0f,
    });
    const auto step = [&](const int n = 1) {
        for (int i = 0; i < n; i++) {
            now_us += 1000;
            vt13.update();
        }
    };

    // 上电未连接：全部为 0
    step();
    assert(!vt13.is_connect && !vt13.is_enable && vt13.rc.mode == VT13::ERR);
    assert(vt13.vx == 0 && vt13.bullet_speed == 0 && vt13.bullet_freq == 0);

    // 帧解析：N 挡、右摇杆上推满量程、左摇杆上推半量程
    Input in;
    in.ch[1] = 1024 + 660;
    in.ch[2] = 1024 + 330;
    send(in);
    step();
    assert(vt13.is_connect && vt13.is_enable && vt13.rc.mode == VT13::N);
    assert(near(vt13.rc.x, 1) && near(vt13.vx, 2) && vt13.vy == 0);
    assert(near(vt13.pitch_speed, -1.5f) && vt13.bullet_speed == 0 && vt13.bullet_freq == 0);

    // W 斜坡：摇杆归零后按 10 m/s^2 加速 100 ms
    in = {};
    in.key = 1u << 0;
    send(in);
    step(100);
    assert(near(vt13.vx, 1.0f));

    // 正速度时按 S：正速度清零后反向加速
    in.key = 1u << 1;
    send(in);
    step();
    assert(near(vt13.vx, -0.01f));
    step(99);
    assert(near(vt13.vx, -1.0f));

    // 松手按 5 m/s^2 减速回零，不越过 0
    in.key = 0;
    send(in);
    step(100);
    assert(near(vt13.vx, -0.5f));
    step(200);
    assert(vt13.vx == 0);

    // 键盘与摇杆叠加后限幅
    in.key = 1u << 0;
    in.ch[1] = 1024 + 660;
    send(in);
    step(300);
    assert(near(vt13.vx, 2.0f));
    in.ch[1] = 1024 - 660;
    send(in);
    step();
    assert(near(vt13.vx, 0.0f));

    // A/D 平移、Shift/Ctrl 旋转
    in = {};
    in.key = 1u << 2 | 1u << 4;
    send(in);
    step(50);
    assert(near(vt13.vy, 0.5f) && near(vt13.wr, 1.0f));
    in.key = 1u << 3 | 1u << 5;
    send(in);
    step(10);
    assert(near(vt13.vy, -0.1f) && near(vt13.wr, -0.2f));
    in.key = 0;
    send(in);
    step(5);
    assert(near(vt13.wr, 0.0f) && near(vt13.vy, -0.075f));

    // 拨轮与鼠标；鼠标超量程时限幅
    in = {};
    in.wheel = 1024 - 330;
    in.ch[3] = 1024 - 330;
    in.mouse_x = -250;
    in.mouse_y = 5000;
    send(in);
    step();
    assert(near(vt13.wr, 2.0f) && near(vt13.yaw_speed, 6.0f) && near(vt13.pitch_speed, -3.0f));

    // trigger：pitch 摇杆改为弹频倍率，鼠标 pitch 仍控制云台
    in = {};
    in.trigger = true;
    in.ch[2] = 1024 - 330;
    in.mouse_y = 250;
    send(in);
    step();
    assert(near(vt13.bullet_freq, -5.0f) && near(vt13.pitch_speed, -1.5f));

    // 鼠标左键按最高弹频；trigger 优先
    in = {};
    in.left = true;
    send(in);
    step();
    assert(near(vt13.bullet_freq, 10.0f));
    in.trigger = true;
    send(in);
    step();
    assert(vt13.bullet_freq == 0);

    // app 修改最高弹频后下次 update 生效
    in = {};
    in.left = true;
    send(in);
    vt13.config.bullet_freq = 15.0f;
    step();
    assert(near(vt13.bullet_freq, 15.0f));

    // S 挡开摩擦轮，C 挡失能
    in = {};
    in.mode = 2;
    send(in);
    step();
    assert(vt13.is_enable && vt13.bullet_speed == 25.0f);
    in.mode = 0;
    send(in);
    step();
    assert(!vt13.is_enable && vt13.bullet_speed == 0);

    // 自瞄：photo 或鼠标右键
    in = {};
    in.photo = true;
    send(in);
    step();
    assert(vt13.is_auto_aim);
    in = {};
    in.right = true;
    send(in);
    step();
    assert(vt13.is_auto_aim);
    in.right = false;
    send(in);
    step();
    assert(!vt13.is_auto_aim);

    // 断联：原始量、斜坡与操控量清零
    in = {};
    in.mode = 2;
    in.key = 1u << 0 | 1u << 4;
    in.left = true;
    send(in);
    step(50);
    assert(vt13.vx > 0 && vt13.wr > 0);
    step(500);
    assert(!vt13.is_connect && !vt13.is_enable && vt13.rc.mode == VT13::ERR);
    assert(vt13.vx == 0 && vt13.vy == 0 && vt13.wr == 0 && vt13.yaw_speed == 0 && vt13.pitch_speed == 0);
    assert(vt13.bullet_speed == 0 && vt13.bullet_freq == 0 && !vt13.is_auto_aim && !vt13.key.w);

    // 重连后斜坡从 0 开始
    in.key = 1u << 0;
    send(in);
    step();
    assert(near(vt13.vx, 0.01f));

    // 加减速度为无穷大：按键一个周期到满量程，反向直接到反向满量程，松手一个周期归零
    VT13 instant({
        .uart = Uart(6),
        .vxy_max = 2.0f,
        .axy = INFINITY,
        .dxy = INFINITY,
        .wr_max = 4.0f,
        .ar = INFINITY,
        .dr = INFINITY,
    });
    const auto step_instant = [&] {
        now_us += 1000;
        instant.update();
    };
    in = {};
    in.key = 1u << 0 | 1u << 2 | 1u << 4;
    send(in);
    step_instant();
    assert(instant.vx == 2.0f && instant.vy == 2.0f && instant.wr == 4.0f);
    in.key = 1u << 1 | 1u << 3 | 1u << 5;
    send(in);
    step_instant();
    assert(instant.vx == -2.0f && instant.vy == -2.0f && instant.wr == -4.0f);
    in.key = 0;
    send(in);
    step_instant();
    assert(instant.vx == 0 && instant.vy == 0 && instant.wr == 0);
    step_instant();
    assert(instant.vx == 0 && instant.vy == 0 && instant.wr == 0);

    // 0x0311：遥控帧与图传链路裁判帧混在同一串口；只回调 0x0311，坏帧与其他命令码跳过，后续遥控帧照常解析
    VT13 mixed({.uart = Uart(6)});
    int command_count = 0;
    std::vector<uint8_t> command;
    mixed.add_0311_callback([&](const uint8_t* data, const size_t size) {
        command_count++;
        command.assign(data, data + size);
    });
    uint8_t payload[60];
    for (size_t i = 0; i < sizeof(payload); i++)
        payload[i] = static_cast<uint8_t>(0xA5 + i);
    in = {};
    in.mode = 0;
    uint8_t rc_frame[21];
    build(in, rc_frame);
    std::vector<uint8_t> stream(rc_frame, rc_frame + 21);
    append_referee(stream, 0x0311, payload, 60);
    append_referee(stream, 0x0310, payload, 10);
    append_referee(stream, 0x0311, payload, 5);
    stream[stream.size() - 1] ^= 0xFF; // 破坏最后一帧 CRC16
    in.mode = 2;
    build(in, rc_frame);
    stream.insert(stream.end(), rc_frame, rc_frame + 21);

    // 逐段送入，模拟空闲中断分包
    for (size_t pos = 0; pos < stream.size(); pos += 7)
        uart_callback(stream.data() + pos, std::min<size_t>(7, stream.size() - pos));
    now_us += 1000;
    mixed.update();
    assert(command_count == 1 && command.size() == 60 && std::memcmp(command.data(), payload, 60) == 0);
    assert(mixed.is_connect && mixed.rc.mode == VT13::S);

    // 超过 60 字节的载荷不回调
    stream.clear();
    uint8_t long_payload[61]{};
    append_referee(stream, 0x0311, long_payload, 61);
    uart_callback(stream.data(), stream.size());
    assert(command_count == 1);

    std::puts("PASS: frame parse, key ramps, infinite ramps, gimbal speed, shooter, auto aim, disconnect and 0x0311");
}
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stubs.cpp").write_text(STUBS, encoding="utf-8")
    (OUT / "test.cpp").write_text(TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "lib"),
                    str(OUT / "stubs.cpp"), str(OUT / "test.cpp"), str(ROOT / "lib/rc/vt13.cpp"),
                    str(ROOT / "lib/referee/crc.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
