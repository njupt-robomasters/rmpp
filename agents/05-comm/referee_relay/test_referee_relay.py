"""RefereeRelay 主机测试：用 Timer、Uart、Can、IrqLock 替身编译实际 lib/referee/referee_relay.cpp、referee.cpp、referee_sender.cpp 与 CRC，
下板 RefereeRelay 接 Can(1)，上板 Referee、RefereeSender 接 Can(2)，由测试搬运两路 CAN 帧，覆盖下板 0x00 分包解析、上板 0x01 拼接、链路忙、ID 过滤与缓冲区溢出。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/tmp/referee_relay/referee_relay"

STUBS = r"""
#include "bsp/can.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include <map>
#include <vector>

uint64_t now_us = 1000000;
std::map<int, std::vector<Uart::RxCallback>> uart_callbacks;
std::map<int, std::vector<uint8_t>> uart_sent;
bool uart_busy = false;
std::map<int, std::vector<Can::RxCallback>> can_callbacks;
std::map<int, std::vector<CanFrame>> can_sent;
bool can_busy = false;

float Timer::elapsed() const { return static_cast<float>(now_us - last_cycles) * 1e-6f; }
float Timer::restart() {
    const float ret = elapsed();
    last_cycles = now_us;
    return ret;
}

bool Uart::can_send() const { return !uart_busy; }
void Uart::send(const uint8_t* data, const std::size_t size) const {
    if (uart_busy || size == 0 || size > 512)
        return;
    uart_sent[id].insert(uart_sent[id].end(), data, data + size);
}
void Uart::add_rx_callback(RxCallback callback) const { uart_callbacks[id].push_back(callback); }

bool Can::can_send() const { return !can_busy; }
void Can::send(const CanFrame& frame) const {
    if (!can_busy)
        can_sent[id].push_back(frame);
}
void Can::add_rx_callback(RxCallback callback) const { can_callbacks[id].push_back(callback); }

IrqLock::IrqLock() : primask(0) {}
IrqLock::~IrqLock() {}
"""

TESTS = r"""
#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>
#include "referee/crc.hpp"
#include "referee/referee.hpp"
#include "referee/referee_relay.hpp"
#include "referee/referee_sender.hpp"

extern uint64_t now_us;
extern std::map<int, std::vector<Uart::RxCallback>> uart_callbacks;
extern std::map<int, std::vector<uint8_t>> uart_sent;
extern bool uart_busy;
extern std::map<int, std::vector<Can::RxCallback>> can_callbacks;
extern std::map<int, std::vector<CanFrame>> can_sent;
extern bool can_busy;

// 组裁判协议帧：0xA5 帧头、CRC8、命令码、载荷、CRC16
static std::vector<uint8_t> make_frame(const uint16_t cmd_id, const void* data, const size_t size) {
    std::vector<uint8_t> frame{0xA5, static_cast<uint8_t>(size), static_cast<uint8_t>(size >> 8), 0};
    frame.push_back(crc8_referee(frame.data(), 4));
    frame.push_back(static_cast<uint8_t>(cmd_id));
    frame.push_back(static_cast<uint8_t>(cmd_id >> 8));
    const auto* bytes = static_cast<const uint8_t*>(data);
    frame.insert(frame.end(), bytes, bytes + size);
    const uint16_t crc = crc16_referee(frame.data(), static_cast<uint32_t>(frame.size()));
    frame.push_back(static_cast<uint8_t>(crc));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    return frame;
}

// 串口接收：按 chunk 分段调用回调，模拟空闲中断分包
static void uart_receive(const int id, const std::vector<uint8_t>& data, const size_t chunk) {
    for (size_t pos = 0; pos < data.size(); pos += chunk)
        for (const auto& callback : uart_callbacks[id])
            callback(data.data() + pos, std::min(chunk, data.size() - pos));
}

// 搬运 from 口已发出的 CAN 帧到 to 口的接收回调，返回帧数
static size_t deliver(const int from, const int to) {
    const auto frames = can_sent[from];
    can_sent[from].clear();
    for (const auto& frame : frames)
        for (const auto& callback : can_callbacks[to])
            callback(frame);
    return frames.size();
}

int main() {
    RefereeRelay relay({.uart = Uart(1), .can = Can(1)});
    Referee referee({.can = Can(2)});
    RefereeSender sender({.can = Can(2)});

    // 下板 0x00：两帧裁判报文按 5 字节分包进入串口，每次 send() 发至多 8 字节，上板 Referee 解析
    RobotStatus status{};
    status.robot_id = 3;
    status.chassis_power_limit = 80;
    PowerHeatData power{};
    power.buffer_energy = 42;
    std::vector<uint8_t> to_can = make_frame(0x0201, &status, sizeof(status));
    const auto power_frame = make_frame(0x0202, &power, sizeof(power));
    to_can.insert(to_can.end(), power_frame.begin(), power_frame.end());
    uart_receive(1, to_can, 5);

    size_t up_frames = 0, up_bytes = 0;
    for (int i = 0; i < 100; i++) {
        relay.send();
        for (const auto& frame : can_sent[1]) {
            assert(frame.id == 0x00 && !frame.is_extend && frame.size >= 1 && frame.size <= 8);
            up_bytes += frame.size;
        }
        up_frames += deliver(1, 2);
    }
    assert(up_bytes == to_can.size() && up_frames == (to_can.size() + 7) / 8);
    assert(referee.is_connect && referee.robot_status.robot_id == 3 && referee.robot_status.chassis_power_limit == 80);
    assert(referee.power_heat_data.buffer_energy == 42);

    // CAN 忙时不发且保留数据，空闲后按序续发
    const auto status2 = [&] {
        RobotStatus s{};
        s.robot_id = 103;
        return make_frame(0x0201, &s, sizeof(s));
    }();
    uart_receive(1, status2, 64);
    can_busy = true;
    relay.send();
    assert(can_sent[1].empty());
    can_busy = false;
    for (int i = 0; i < 10; i++) {
        relay.send();
        deliver(1, 2);
    }
    assert(referee.robot_status.robot_id == 103);

    // 上板 0x01：RefereeSender 每次 send() 发 1 帧，下板拼接后由串口原样发出
    // 本机 103（蓝方 3 号步兵），绘制一个图形发往选手端 0x0167
    sender.set_robot_id(referee.robot_status.robot_id);
    const Figure figure = RefereeSender::line(0x000100, 0, RefereeSender::WHITE, 2, 960, 660, 960, 890);
    assert(sender.draw(&figure, 1, RefereeSender::ADD));
    uint8_t interaction[6 + 15] = {0x01, 0x01, 103, 0, 0x67, 0x01};
    std::memcpy(interaction + 6, &figure, sizeof(figure));
    interaction[6 + 3] = (interaction[6 + 3] & ~0x07) | RefereeSender::ADD;
    const auto expected = make_frame(0x0301, interaction, sizeof(interaction));

    // 串口忙时只缓存，空闲后一次发出
    uart_busy = true;
    for (int i = 0; i < 10; i++) {
        sender.send();
        deliver(2, 1);
        relay.send();
    }
    assert(uart_sent[1].empty());
    uart_busy = false;
    relay.send();
    assert(uart_sent[1].size() == expected.size());
    assert(std::memcmp(uart_sent[1].data() + 5, expected.data() + 5, expected.size() - 5) == 0); // seq 由 RefereeSender 维护
    assert(uart_sent[1][0] == 0xA5 && crc8_referee(uart_sent[1].data(), 4) == uart_sent[1][4]);
    uart_sent[1].clear();

    // 下板只收 0x01 帧
    for (const auto& callback : can_callbacks[1])
        callback(CanFrame{.id = 0x02, .size = 3, .data = {1, 2, 3}});
    relay.send();
    assert(uart_sent[1].empty());

    // to_uart 溢出：320 B 丢掉最早的 64 B，按序发出最新 256 B
    uart_busy = true;
    for (size_t offset = 0; offset < 320; offset += 8) {
        CanFrame frame{.id = 0x01, .size = 8};
        for (size_t index = 0; index < frame.size; index++)
            frame.data[index] = static_cast<uint8_t>(offset + index);
        for (const auto& callback : can_callbacks[1])
            callback(frame);
    }
    uart_busy = false;
    relay.send();
    assert(uart_sent[1].size() == 256);
    for (size_t index = 0; index < uart_sent[1].size(); index++)
        assert(uart_sent[1][index] == static_cast<uint8_t>(64 + index));

    // to_can 溢出：1200 B 按 400 B 分包进入，仅转发最后 1024 B
    std::vector<uint8_t> flood(1200);
    for (size_t index = 0; index < flood.size(); index++)
        flood[index] = static_cast<uint8_t>(index);
    uart_receive(1, flood, 400);
    std::vector<uint8_t> forwarded;
    for (int i = 0; i < 200; i++) {
        relay.send();
        for (const auto& frame : can_sent[1])
            forwarded.insert(forwarded.end(), frame.data, frame.data + frame.size);
        can_sent[1].clear();
    }
    assert(forwarded == std::vector<uint8_t>(flood.end() - 1024, flood.end()));

    std::puts("PASS: up split and parse, can busy, down concat, uart busy, id filter, overflow");
}
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stubs.cpp").write_text(STUBS, encoding="utf-8")
    (OUT / "test.cpp").write_text(TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-Wno-packed-bitfield-compat",
                    "-I", str(ROOT / "lib"), str(OUT / "stubs.cpp"), str(OUT / "test.cpp"),
                    str(ROOT / "lib/referee/referee_relay.cpp"), str(ROOT / "lib/referee/referee.cpp"),
                    str(ROOT / "lib/referee/referee_sender.cpp"), str(ROOT / "lib/referee/crc.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
