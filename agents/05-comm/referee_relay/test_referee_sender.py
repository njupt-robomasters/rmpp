"""RefereeSender 与 RobotUi 主机测试：用 Timer、Can 替身编译实际 lib/referee/referee_sender.cpp、robot_ui.cpp 与 CRC，
覆盖图形位段编码（与协议表逐位比对）、组帧与 CRC、0x0301 30 Hz 间隔、0x0308 不限频、本机 ID 未知暂停、无选手端丢弃 UI、
MODIFY 合并、clear_ui 保留其他报文、CAN 忙续发，以及 RobotUi 初始布局与仪表刷新。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/referee_relay/referee_sender"

STUBS = r"""
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <map>
#include <vector>

uint64_t now_us = 1000000;
std::map<int, std::vector<CanFrame>> can_sent;
bool can_busy = false;

float Timer::elapsed() const { return static_cast<float>(now_us - last_cycles) * 1e-6f; }
float Timer::restart() {
    const float ret = elapsed();
    last_cycles = now_us;
    return ret;
}

bool Can::can_send() const { return !can_busy; }
void Can::send(const CanFrame& frame) const { can_sent[id].push_back(frame); }
void Can::add_rx_callback(RxCallback) const {}
"""

TESTS = r"""
#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>
#include "math/unit.hpp"
#include "referee/crc.hpp"
#include "referee/robot_ui.hpp"

extern uint64_t now_us;
extern std::map<int, std::vector<CanFrame>> can_sent;
extern bool can_busy;

struct Frame {
    uint16_t cmd_id;
    std::vector<uint8_t> payload;
};

// 调用 send() n 次，每次推进 1 ms，拼接 Can(2) 上 tx_id 1 的字节并按帧头切分校验
static std::vector<Frame> run(RefereeSender& sender, const int n) {
    std::vector<uint8_t> bytes;
    for (int i = 0; i < n; i++) {
        now_us += 1000;
        sender.send();
        for (const auto& frame : can_sent[2]) {
            assert(frame.id == 1 && frame.size >= 1 && frame.size <= 8);
            bytes.insert(bytes.end(), frame.data, frame.data + frame.size);
        }
        can_sent[2].clear();
    }
    std::vector<Frame> frames;
    size_t pos = 0;
    while (pos < bytes.size()) {
        assert(bytes[pos] == 0xA5 && crc8_referee(bytes.data() + pos, 4) == bytes[pos + 4]);
        const size_t size = bytes[pos + 1] | (bytes[pos + 2] << 8);
        assert(pos + size + 9 <= bytes.size());
        const uint16_t crc = bytes[pos + size + 7] | (bytes[pos + size + 8] << 8);
        assert(crc16_referee(bytes.data() + pos, size + 7) == crc);
        frames.push_back({static_cast<uint16_t>(bytes[pos + 5] | (bytes[pos + 6] << 8)),
                          std::vector<uint8_t>(bytes.begin() + pos + 7, bytes.begin() + pos + 7 + size)});
        pos += size + 9;
    }
    return frames;
}

static uint16_t u16(const std::vector<uint8_t>& data, const size_t pos) {
    return static_cast<uint16_t>(data[pos] | (data[pos + 1] << 8));
}

// 按协议表 1-27 手工拼 15 B 图形，与位段结构体逐字节比对
static void expect_figure(const Figure& figure, const uint32_t name, const uint32_t op, const uint32_t type, const uint32_t layer,
                          const uint32_t color, const uint32_t a, const uint32_t b, const uint32_t width, const uint32_t x,
                          const uint32_t y, const uint32_t c, const uint32_t d, const uint32_t e) {
    uint8_t expected[15]{static_cast<uint8_t>(name), static_cast<uint8_t>(name >> 8), static_cast<uint8_t>(name >> 16)};
    const uint32_t words[] = {op | type << 3 | layer << 6 | color << 10 | a << 14 | b << 23, width | x << 10 | y << 21, c | d << 10 | e << 21};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            expected[3 + i * 4 + j] = static_cast<uint8_t>(words[i] >> (j * 8));
    assert(std::memcmp(&figure, expected, 15) == 0);
}

int main() {
    static_assert(sizeof(Figure) == 15 && sizeof(SentryCommand) == 4);

    // 图形编码：椭圆、圆弧半轴在 details_d/e，圆弧角度换算为度并保留整圈 360，整数按补码拆分
    expect_figure(RefereeSender::line(0x123456, 3, RefereeSender::WHITE, 2, 960, 660, 960, 890), 0x123456, 0, 0, 3, 8, 0, 0, 2, 960, 660, 0, 960, 890);
    expect_figure(RefereeSender::circle(1, 1, RefereeSender::GREEN, 2, 960, 540, 21), 1, 0, 2, 1, 2, 0, 0, 2, 960, 540, 21, 0, 0);
    expect_figure(RefereeSender::ellipse(2, 1, RefereeSender::GREEN, 2, 100, 200, 30, 25), 2, 0, 3, 1, 2, 0, 0, 2, 100, 200, 0, 30, 25);
    expect_figure(RefereeSender::arc(3, 1, RefereeSender::PINK, 10, 960, 540, -20 * deg, 360 * deg, 85, 85), 3, 0, 4, 1, 5, 340, 360, 10, 960, 540, 0, 85, 85);
    const uint32_t minus = static_cast<uint32_t>(-1234);
    expect_figure(RefereeSender::integer(4, 2, RefereeSender::WHITE, 2, 615, 565, 15, -1234), 4, 0, 6, 2, 8, 15, 0, 2, 615, 565, minus & 0x3FF, (minus >> 10) & 0x7FF, minus >> 21);
    expect_figure(RefereeSender::floating(5, 2, RefereeSender::WHITE, 2, 0, 0, 15, 1.234f), 5, 0, 5, 2, 8, 15, 0, 2, 0, 0, 1234, 0, 0);

    // 本机 ID 未知时暂停；无选手端（哨兵 7）丢弃 UI，0x0120 发往服务器 0x8080
    RefereeSender sender({.can = Can(2)});
    assert(sender.delete_all());
    SentryCommand command{};
    assert(command.posture == 3);
    command.confirm_revive = 1;
    command.posture = 2;
    command.activate_rune = 1;
    assert(sender.send_sentry_command(command));
    assert(run(sender, 100).empty());
    sender.set_robot_id(7);
    auto frames = run(sender, 100);
    assert(frames.size() == 1 && frames[0].cmd_id == 0x0301 && frames[0].payload.size() == 10);
    assert(u16(frames[0].payload, 0) == 0x0120 && u16(frames[0].payload, 2) == 7 && u16(frames[0].payload, 4) == 0x8080);
    assert(frames[0].payload[6] == 1 && frames[0].payload[7] == 0 && frames[0].payload[8] == 0xC0 && frames[0].payload[9] == 0); // 姿态 bit 21~22，能量机关 bit 23

    // 0x0308：小端 UTF-16 补 0 到 30 B，发送者、接收者在前；与 0x0301 交替时不受 30 Hz 约束
    sender.set_robot_id(103);
    const char16_t text[] = u"哨兵A";
    assert(!sender.send_custom_info(0x0167, text, 16));
    assert(sender.send_custom_info(0x0167, text, 3));
    frames = run(sender, 20);
    assert(frames.size() == 1 && frames[0].cmd_id == 0x0308 && frames[0].payload.size() == 34);
    assert(u16(frames[0].payload, 0) == 103 && u16(frames[0].payload, 2) == 0x0167);
    assert(u16(frames[0].payload, 4) == u'哨' && u16(frames[0].payload, 6) == u'兵' && u16(frames[0].payload, 8) == u'A' && frames[0].payload[10] == 0);

    // 0x0301 间隔至少 1/30 s：3 个删除图层在 100 ms 内至多发出 3 帧且间隔不小于 33 ms
    for (uint8_t layer = 0; layer < 3; layer++)
        assert(sender.delete_layer(layer));
    assert(!sender.delete_layer(10));
    frames = run(sender, 34);
    assert(frames.size() == 1 && u16(frames[0].payload, 0) == 0x0100 && u16(frames[0].payload, 4) == 0x0167);
    frames = run(sender, 33);
    assert(frames.size() == 1);
    frames = run(sender, 40);
    assert(frames.size() == 1 && frames[0].payload[6] == 1 && frames[0].payload[7] == 2);

    // MODIFY 合并：同子内容、首图形同名才替换；ADD 不合并；clear_ui 只删 UI，保留 0x0120
    const Figure first = RefereeSender::line(0x160, 1, RefereeSender::GREEN, 2, 0, 0, 1, 1);
    Figure moved = first;
    moved.x = 100;
    const Figure other = RefereeSender::line(0x170, 1, RefereeSender::GREEN, 2, 0, 0, 1, 1);
    assert(sender.draw(&first, 1, RefereeSender::ADD));
    assert(sender.draw(&first, 1, RefereeSender::MODIFY));
    assert(sender.draw(&moved, 1, RefereeSender::MODIFY));
    assert(sender.draw(&other, 1, RefereeSender::MODIFY));
    frames = run(sender, 200);
    assert(frames.size() == 3);
    assert((frames[0].payload[6 + 3] & 7) == RefereeSender::ADD);
    Figure merged;
    std::memcpy(&merged, frames[1].payload.data() + 6, sizeof(merged));
    assert(merged.operation == RefereeSender::MODIFY && merged.x == 100);
    assert(frames[2].payload[6] == 0x70);
    assert(sender.draw(&first, 1, RefereeSender::ADD));
    assert(sender.send_sentry_command({}));
    assert(sender.delete_all());
    sender.clear_ui();
    frames = run(sender, 100);
    assert(frames.size() == 1 && u16(frames[0].payload, 0) == 0x0120);

    // CAN 忙时保留已组好的帧；后续 21 个请求仅保留最新 20 个
    can_busy = true;
    assert(sender.delete_all());
    assert(run(sender, 50).empty());
    for (uint8_t index = 0; index < 21; index++)
        assert(sender.delete_layer(index % 10));
    can_busy = false;
    frames = run(sender, 1500);
    assert(frames.size() == 21 && frames[0].payload[6] == 2);
    for (size_t index = 1; index < frames.size(); index++)
        assert(frames[index].payload[6] == 1 && frames[index].payload[7] == index % 10);

    // RobotUi：init() 排入 11 个初始布局，update 每周期排入的仪表只保留最新 2 份，按 30 Hz 全部发出
    RefereeSender ui_sender({.can = Can(2)});
    RobotUi ui({});
    ui.rub_left_stage2 = ui.CYAN;
    ui.init(ui_sender);
    ui.yaw_angle = 30 * deg;
    ui.pitch_angle = -10 * deg;
    ui.chassis_wr = 60 * rpm;
    ui.rub_left_stage1 = ui.ORANGE;
    ui.rub_right_stage1 = ui.PINK;
    ui.rub_right_stage2 = ui.GREEN;
    for (int i = 0; i < 1000; i++) {
        ui.bullet_freq = static_cast<float>(i % 20);
        ui.update(ui_sender);
    }
    ui_sender.set_robot_id(3);
    frames = run(ui_sender, 1000);
    assert(frames.size() == 16);
    std::vector<uint16_t> ids;
    for (const auto& frame : frames) {
        assert(frame.cmd_id == 0x0301 && u16(frame.payload, 2) == 3 && u16(frame.payload, 4) == 0x0103);
        ids.push_back(u16(frame.payload, 0));
    }
    const std::vector<uint16_t> expected_ids = {0x0103, 0x0104, 0x0110, 0x0110, 0x0110, 0x0110, 0x0104, 0x0104, 0x0104, 0x0104, 0x0103, 0x0104, 0x0104, 0x0104, 0x0104, 0x0103};
    assert(ids == expected_ids);
    Figure modules[7], tail[5];
    std::memcpy(modules, frames[14].payload.data() + 6, sizeof(modules));
    std::memcpy(tail, frames[15].payload.data() + 6, sizeof(tail));
    assert(modules[5].color == ui.ORANGE && tail[0].color == ui.CYAN);
    assert(modules[6].color == ui.PINK && tail[1].color == ui.GREEN);
    assert(modules[5].x < tail[0].x && modules[6].x < tail[1].x);
    for (int index = 8; index <= 10; index++) {
        const int count = index == 10 ? 4 : 7;
        for (int slot = 0; slot < count; slot++) {
            Figure figure;
            std::memcpy(&figure, frames[index].payload.data() + 6 + slot * sizeof(Figure), sizeof(Figure));
            assert(figure.operation == RefereeSender::ADD && figure.color == ui.WHITE);
        }
    }

    // 末两帧为最新仪表：数值 yaw 30 度、pitch -10 度、转速 60 rpm、弹频 19；浮点换算后截断允许差 1
    Figure numbers[7];
    std::memcpy(numbers, frames[12].payload.data() + 6, sizeof(numbers));
    assert(numbers[0].name[0] == 0x70 && numbers[0].operation == RefereeSender::MODIFY);
    const auto value = [](const Figure& figure) {
        return static_cast<int32_t>(static_cast<uint32_t>(figure.details_c) | static_cast<uint32_t>(figure.details_d) << 10 | static_cast<uint32_t>(figure.details_e) << 21);
    };
    const auto near = [](const int32_t a, const int32_t b) { return a - b <= 1 && b - a <= 1; };
    assert(near(value(numbers[1]), 60) && value(numbers[2]) == 19 && near(value(numbers[4]), 30) && near(value(numbers[5]), -10));

    std::puts("PASS: figure encoding, framing, 30 Hz, 0x0308, robot id, no client drop, modify merge, clear_ui, can busy, robot ui");
}
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stubs.cpp").write_text(STUBS, encoding="utf-8")
    (OUT / "test.cpp").write_text(TESTS, encoding="utf-8")
    executable = OUT / "test.exe"
    subprocess.run(["g++", "-std=c++20", "-mno-ms-bitfields", "-Wall", "-Wextra", "-Werror", "-Wno-packed-bitfield-compat",
                    "-I", str(ROOT / "lib"), "-I", str(ROOT / "lib/third_party/eigen-3.4.0"), str(OUT / "stubs.cpp"), str(OUT / "test.cpp"),
                    str(ROOT / "lib/referee/referee_sender.cpp"), str(ROOT / "lib/referee/robot_ui.cpp"),
                    str(ROOT / "lib/referee/crc.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
