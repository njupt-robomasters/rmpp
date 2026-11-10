// mc02 CAN 发送上板测试：TX Queue 模式，按 ID 优先级发送；同 ID 只留最新；满时撤销最早写入，正在发送时改撤次早
// 接线：CAN1 与 CAN2 短接，两端板载 60 Ω
// 结果全部放在全局变量，测完停在空循环，由调试器读取
#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// FDCAN1 寄存器（app 不包含 HAL，按地址访问）
static volatile uint32_t& CCCR = *reinterpret_cast<volatile uint32_t*>(0x4000A018);
static volatile uint32_t& TXFQS = *reinterpret_cast<volatile uint32_t*>(0x4000A0C4);
static volatile uint32_t& TXBRP = *reinterpret_cast<volatile uint32_t*>(0x4000A0CC);
static volatile uint32_t& TXBCR = *reinterpret_cast<volatile uint32_t*>(0x4000A0D4);
static volatile uint32_t& CYCCNT = *reinterpret_cast<volatile uint32_t*>(0xE0001004);

struct Snapshot {
    uint32_t tffl, tfgi, tfqpi, full, txbrp, can_send;
};

struct Rx {
    uint32_t id;
    uint32_t d0;
};

static Can can1(1);
static Can can2(2);

// 观察量
volatile Snapshot snap[12];
volatile Rx rx[160];
volatile uint32_t rx_count = 0;
volatile uint32_t mark[8];   // 各组结束时的 rx_count
volatile uint32_t cycles[8]; // 耗时，CPU 周期（520 MHz）
volatile uint32_t stage = 0; // 完成到第几组，99 表示全部完成

static void take(const int i) {
    const uint32_t s = TXFQS;
    snap[i].tffl = s & 0x3F;
    snap[i].tfgi = (s >> 8) & 0x1F;
    snap[i].tfqpi = (s >> 16) & 0x1F;
    snap[i].full = (s >> 21) & 1;
    snap[i].txbrp = TXBRP;
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
    take(0); // 空闲时的状态寄存器

    // A 暂停发送：置 INIT 后按 ID 降序写满 32 帧（0x11F 最先写入）
    CCCR = CCCR | 1u;
    while (!(CCCR & 1u)) {
    }
    for (uint32_t i = 0; i < 32; i++)
        send(0x11F - i, i);
    take(1);

    // A1 同 ID 0x105：只撤销旧 0x105，新帧写入它空出的槽位
    send(0x105, 0xA5);
    take(2);

    // A2 新 ID 0x200：满 → 撤销最早写入的 0x11F
    send(0x200, 0xB0);
    take(3);

    // A3 手动撤销槽位 16（0x10F），看满标志是否立即清除、写指针是否指向它
    TXBCR = 1u << 16;
    Timer::delay(0.001f);
    take(4);

    // A4 再写 0x201：应直接写入槽位 16，不撤销其他帧
    send(0x201, 0xB1);
    take(5);
    stage = 1;

    // 退出 INIT：应按 ID 升序发出
    CCCR = CCCR & ~1u;
    Timer::delay(0.02f);
    take(6);
    mark[0] = rx_count;
    stage = 2;

    // B 总线运行：同 ID 0x300 连写 10 帧，第 5 帧后夹一帧 0x301
    for (uint32_t i = 0; i < 10; i++) {
        send(0x300, i);
        if (i == 4)
            send(0x301, 0xC1);
    }
    Timer::delay(0.02f);
    mark[1] = rx_count;
    stage = 3;

    // C 总线运行：按 ID 降序连写 32 帧，0x41F 写入即开始发送
    uint32_t t0 = CYCCNT;
    for (uint32_t i = 0; i < 32; i++)
        send(0x41F - i, i);
    cycles[0] = CYCCNT - t0; // 写 32 帧总耗时
    take(7);

    // 第 33 帧：最早的 0x41F 正在发送，应改撤次早的 0x41E 后写入
    t0 = CYCCNT;
    send(0x3FF, 0xD0);
    cycles[1] = CYCCNT - t0;
    take(8);
    Timer::delay(0.02f);
    mark[2] = rx_count;
    stage = 4;

    // D 不满时一次 send 的耗时（总线空闲）
    t0 = CYCCNT;
    send(0x500, 0);
    cycles[2] = CYCCNT - t0;
    // 同 ID 且旧帧正在发送：撤销不了，队列不满，不应等待
    t0 = CYCCNT;
    send(0x500, 1);
    cycles[3] = CYCCNT - t0;
    Timer::delay(0.02f);
    mark[3] = rx_count;
    take(9);

    stage = 99;
    while (true) {
    }
}
