// C 板 CAN 发送上板测试：3 个邮箱按 ID 优先级发送；同 ID 只留最新；满时撤销最早写入，正在发送时改撤次早
// 接线：CAN1 发、CAN2 收；未短接时 B/C 组收不到帧，只看寄存器
// 结果全部放在全局变量，测完停在空循环，由调试器读取
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// CAN1 寄存器（app 不包含 HAL，按地址访问）
static volatile uint32_t& MCR = *reinterpret_cast<volatile uint32_t*>(0x40006400);
static volatile uint32_t& MSR = *reinterpret_cast<volatile uint32_t*>(0x40006404);
static volatile uint32_t& TSR = *reinterpret_cast<volatile uint32_t*>(0x40006408);
static volatile uint32_t& ESR = *reinterpret_cast<volatile uint32_t*>(0x40006418);
static volatile uint32_t& CYCCNT = *reinterpret_cast<volatile uint32_t*>(0xE0001004);

static uint32_t tir(const int i) {
    return *reinterpret_cast<volatile uint32_t*>(0x40006580 + 0x10 * i);
}

struct Snapshot {
    uint32_t tme;       // bit0–2 对应邮箱 0–2 为空
    uint32_t id[3];     // 各邮箱 TIR 中的标准 ID
    uint32_t can_send;
};

struct Rx {
    uint32_t id;
    uint32_t d0;
};

static Can can1(1);
static Can can2(2);

// 观察量
volatile Snapshot snap[12];
volatile Rx rx[64];
volatile uint32_t rx_count = 0;
volatile uint32_t mark[8];   // 各组结束时的 rx_count
volatile uint32_t cycles[8]; // 耗时，CPU 周期（168 MHz）
volatile uint32_t esr[4];    // CAN1 ESR：各组结束时的错误计数与最后错误码
volatile uint32_t stage = 0; // 完成到第几组，99 表示全部完成

static void take(const int i) {
    const uint32_t s = TSR;
    snap[i].tme = (s >> 26) & 0x7;
    for (int m = 0; m < 3; m++)
        snap[i].id[m] = tir(m) >> 21;
    snap[i].can_send = can1.can_send();
}

static void send(const uint32_t id, const uint8_t d0) {
    can1.send({.id = id, .size = 8, .data = {d0}});
}

extern "C" void app_main() {
    can2.add_rx_callback([](const CanFrame& frame) {
        if (rx_count < sizeof(rx) / sizeof(rx[0])) {
            rx[rx_count].id = frame.id;
            rx[rx_count].d0 = frame.data[0];
        }
        rx_count = rx_count + 1;
    });
    Timer::delay(0.1f);
    take(0); // 空闲时

    // A 暂停发送：置 INRQ 进入初始化模式，按 ID 降序写满 3 个邮箱（0x103 最先写入）
    MCR = MCR | 1u;
    while (!(MSR & 1u)) {
    }
    send(0x103, 0);
    send(0x102, 1);
    send(0x101, 2);
    take(1);

    // A1 同 ID 0x102：只撤销旧 0x102，新帧写入它空出的邮箱
    uint32_t t0 = CYCCNT;
    send(0x102, 0xA2);
    cycles[0] = CYCCNT - t0;
    take(2);

    // A2 新 ID 0x200：满 → 撤销最早写入的 0x103
    t0 = CYCCNT;
    send(0x200, 0xB0);
    cycles[1] = CYCCNT - t0;
    take(3);
    stage = 1;

    // 退出初始化模式：应按 ID 升序发出 0x101、0x102(A2)、0x200
    MCR = MCR & ~1u;
    while (MSR & 1u) {
    }
    Timer::delay(0.02f);
    take(4);
    mark[0] = rx_count;
    esr[0] = ESR;
    stage = 2;

    // B 总线运行：同 ID 0x300 连写 10 帧，第 5 帧后夹一帧 0x301
    for (uint32_t i = 0; i < 10; i++) {
        send(0x300, i);
        if (i == 4)
            send(0x301, 0xC1);
    }
    Timer::delay(0.02f);
    mark[1] = rx_count;
    esr[1] = ESR;
    stage = 3;

    // C 总线运行：连写 3 帧，0x41F 写入即开始发送；第 4 帧应改撤次早的 0x41E
    send(0x41F, 0);
    send(0x41E, 1);
    send(0x41D, 2);
    take(5);
    t0 = CYCCNT;
    send(0x3FF, 0xD0);
    cycles[2] = CYCCNT - t0;
    take(6);
    Timer::delay(0.02f);
    mark[2] = rx_count;
    esr[2] = ESR;
    stage = 4;

    // D 撤销未在发送的邮箱的生效时间：0x500 在发送，撤销 0x501
    t0 = CYCCNT;
    send(0x500, 0);
    cycles[3] = CYCCNT - t0; // 普通写入耗时
    send(0x501, 1);
    take(7);
    for (int m = 0; m < 3; m++) {
        if (!(TSR & (1u << (26 + m))) && (tir(m) >> 21) == 0x501) {
            t0 = CYCCNT;
            TSR = 1u << (7 + 8 * m);
            while (!(TSR & (1u << (26 + m)))) {
            }
            cycles[4] = CYCCNT - t0;
        }
    }
    Timer::delay(0.02f);
    take(8);
    mark[3] = rx_count;
    esr[3] = ESR;

    stage = 99;
    while (true) {
    }
}
