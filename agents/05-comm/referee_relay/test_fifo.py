"""Fifo 与迁移后环形缓冲主机测试：编译实际 lib/referee/referee.cpp 与 CRC，
覆盖 Fifo 回绕、单元素和批量覆盖队首、溢出计数、peek/pop、[] 读写；Referee 接收环形回绕后连续解析、超长载荷丢弃。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/referee_relay/fifo"

STUBS = r"""
#include "bsp/can.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include <map>
#include <vector>

uint64_t now_us = 1000000;
std::map<int, std::vector<Can::RxCallback>> can_callbacks;
std::map<int, std::vector<CanFrame>> can_sent;

float Timer::elapsed() const { return static_cast<float>(now_us - last_cycles) * 1e-6f; }
float Timer::restart() {
    const float ret = elapsed();
    last_cycles = now_us;
    return ret;
}

bool Uart::can_send() const { return true; }
void Uart::send(const uint8_t*, const std::size_t) const {}
void Uart::add_rx_callback(RxCallback) const {}

bool Can::can_send() const { return true; }
void Can::send(const CanFrame& frame) const { can_sent[id].push_back(frame); }
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
#include "container/fifo.hpp"
#include "referee/crc.hpp"
#include "referee/referee.hpp"

extern uint64_t now_us;
extern std::map<int, std::vector<Can::RxCallback>> can_callbacks;
extern std::map<int, std::vector<CanFrame>> can_sent;

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

// 按 8 字节 CAN 帧送入 id 口
static void can_receive(const int id, const uint32_t frame_id, const std::vector<uint8_t>& data) {
    for (size_t pos = 0; pos < data.size(); pos += 8) {
        CanFrame frame{.id = frame_id, .size = static_cast<uint8_t>(std::min<size_t>(8, data.size() - pos))};
        std::memcpy(frame.data, data.data() + pos, frame.size);
        for (const auto& callback : can_callbacks[id])
            callback(frame);
    }
}

int main() {
    // Fifo：回绕、满时覆盖队首、peek 不取出、pop 丢弃与复制、[] 读写
    Fifo<int, 4> fifo;
    const int a[] = {1, 2, 3};
    assert(fifo.push(a, 3) == 0 && fifo.size == 3);
    fifo.pop(2);
    assert(fifo.size == 1 && fifo[0] == 3);
    const int b[] = {4, 5, 6, 7};
    assert(fifo.push(b, 4) == 1 && fifo.size == 4);
    assert(fifo.push(nullptr, 0) == 0 && fifo[0] == 4);
    fifo.push(8);
    int out[4]{};
    assert(fifo.peek(out, 4) == 4 && out[0] == 5 && out[3] == 8 && fifo.size == 4);
    fifo[1] = 40;
    assert(fifo.pop(out, 2) == 2 && out[0] == 5 && out[1] == 40 && fifo.size == 2);
    assert(fifo[0] == 7 && fifo[1] == 8);
    const int oversized[] = {9, 10, 11, 12, 13, 14};
    assert(fifo.push(oversized, 6) == 4 && fifo.size == 4);
    for (int index = 0; index < 4; index++)
        assert(fifo[index] == 11 + index);
    fifo.pop(10);
    assert(fifo.size == 0);
    for (int value = 9; value <= 18; value++)
        fifo.push(value);
    assert(fifo.size == 4 && fifo.peek(out, 4) == 4);
    for (int index = 0; index < 4; index++)
        assert(out[index] == 15 + index);
    fifo.clear();
    assert(fifo.size == 0 && fifo.peek(out, 4) == 0);

    // Referee：连续 40 帧使 256 B 接收环形多次回绕，每帧均解析
    Referee referee({.can = Can(2)});
    for (uint8_t id = 1; id <= 40; id++) {
        RobotStatus status{};
        status.robot_id = id;
        status.current_HP = static_cast<uint16_t>(id * 10);
        can_receive(2, 0x00, make_frame(0x0201, &status, sizeof(status)));
        assert(referee.robot_status.robot_id == id && referee.robot_status.current_HP == id * 10);
    }
    assert(referee.is_connect);

    // Referee：载荷超过 118 B 的帧头视为错位丢弃，后续有效帧仍解析
    std::vector<uint8_t> stream = make_frame(0x0201, std::vector<uint8_t>(119).data(), 119);
    RobotStatus status{};
    status.robot_id = 3;
    const auto valid = make_frame(0x0201, &status, sizeof(status));
    stream.insert(stream.end(), valid.begin(), valid.end());
    can_receive(2, 0x00, stream);
    assert(referee.robot_status.robot_id == 3);

    std::puts("PASS: fifo wrap/overwrite/drop count/peek/pop/index, referee ring wrap and payload limit");
}
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stubs.cpp").write_text(STUBS, encoding="utf-8")
    (OUT / "test.cpp").write_text(TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-Wno-packed-bitfield-compat",
                    "-I", str(ROOT / "lib"), "-I", str(ROOT / "lib/third_party/eigen-3.4.0"),
                    str(OUT / "stubs.cpp"), str(OUT / "test.cpp"), str(ROOT / "lib/referee/referee.cpp"),
                    str(ROOT / "lib/referee/crc.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
