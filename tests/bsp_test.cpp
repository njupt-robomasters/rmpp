#include "bsp.hpp"
#include "can.h"
#include "imu/lib/bmi088/bmi088_middleware.h"
#include "peripherals.hpp"
#include "rgb/rgb.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

Can global_can(1);
auto global_timer = bsp.timer();
int early_frames = 0;
const bool registered_early = global_can.add_rx_callback([](const CanFrame&) {
    ++early_frames;
    DWT->CYCCNT += SystemCoreClock / 100000;
});
extern "C" void call_bsp_from_c(void);

static void test_can() {
    assert(registered_early && bsp.can(1) && bsp.can(2));
    assert(!bsp.can(0) && !bsp.can(3) && !bsp.can(-1));
    CanFrame frame{.id = 0x123, .data = {1, 2, 3}, .size = 3};
    assert(!bsp.can(99).send(frame));
    assert(!bsp.can(99).add_rx_callback([](const auto&) {}));
    auto copy = global_can;
    assert(copy.send(frame));
    assert(hcan1.sent_header.StdId == 0x123 && hcan1.sent_header.DLC == 3 && hcan1.sent_data[2] == 3);
    hcan1.available = 0;
    const int attempts = hcan1.submissions;
    assert(!global_can.send(frame) && hcan1.submissions == attempts + 1);
    frame.id = 0x800;
    assert(!global_can.send(frame) && hcan1.submissions == attempts + 1);
    frame.id = 0x123456;
    frame.extended = true;
    frame.remote = true;
    hcan1.available = 3;
    assert(global_can.send(frame));
    assert(hcan1.sent_header.ExtId == frame.id && hcan1.sent_header.RTR == CAN_RTR_REMOTE);

    struct Receiver {
        std::vector<int> order;

        void subscribe() {
            bsp.can(1).add_rx_callback([this](const CanFrame& f) { assert(f.size == 2); order.push_back(1); });
            bsp.can(1).add_rx_callback([this](const CanFrame&) { order.push_back(2); });
        }
    };

    static Receiver receiver;
    irq_mask = 1;
    receiver.subscribe();
    assert(irq_mask == 1);
    irq_mask = 0;
    static int can2_frames = 0;
    bsp.can(2).add_rx_callback([](const CanFrame&) { ++can2_frames; });
    for (int i = 0; i < 5; ++i)
        hcan1.received.push_back({{.StdId = 4, .DLC = 2}, {7, 8}});
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(early_frames == 5 && can2_frames == 0 && receiver.order.size() == 10);
    for (unsigned i = 0; i < receiver.order.size(); ++i)
        assert(receiver.order[i] == int(i % 2 + 1));
    hcan2.received.push_back({{.ExtId = 0x1234, .IDE = CAN_ID_EXT, .DLC = 1}, {9}});
    HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
    assert(can2_frames == 1 && early_frames == 5);
    hcan2.ErrorCode = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan2);
}

static void test_uart_usb() {
    assert(bsp.uart(1) && bsp.uart(3) && bsp.uart(6) && !bsp.uart(2));
    std::uint8_t data[] = {10, 20, 30};
    assert(!bsp.uart(2).send({data, 3}) && !bsp.uart(3).send({data, 3}));
    assert(bsp.uart(6).send({data, 3}));
    data[0] = 99;
    assert(huart6.tx != data && huart6.tx[0] == 10);
    assert(!bsp.uart(6).send({data, 3}) && huart6.tx[0] == 10);
    huart6.gState = HAL_UART_STATE_READY;
    assert(!bsp.uart(6).send({data, 257}));
    static std::vector<unsigned> sizes;
    bsp.uart(3).add_rx_callback([](Bytes bytes) { sizes.push_back(bytes.size); });
    bsp.uart(3).add_rx_callback([](Bytes bytes) { sizes.push_back(bytes.size + 1); });
    const auto rearms = huart3.rearms;
    HAL_UARTEx_RxEventCallback(&huart3, 25);
    huart3.event = HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart3, 128);
    assert((sizes == std::vector<unsigned>{25, 26, 128, 129}));
    assert(huart3.rearms == rearms + 2);
    huart3.event = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart3, 64);
    assert(sizes.size() == 4 && huart3.rearms == rearms + 2);
    HAL_UART_ErrorCallback(&huart3);
    assert(huart3.rearms == rearms + 3);
    assert(huart1.rearms == 1);

    assert(bsp.cdc(1) && !bsp.cdc(2));
    assert(!bsp.cdc(1).send({data, 3}));
    static USBD_CDC_HandleTypeDef cdc;
    hUsbDeviceFS = {USBD_STATE_CONFIGURED, &cdc};
    assert(bsp.cdc(1).send({data, 3}));
    data[0] = 77;
    assert(usb_tx != data && usb_tx[0] == 99);
    assert(!bsp.cdc(1).send({data, 3}) && usb_tx[0] == 99);
    cdc.TxState = 0;
    assert(!bsp.cdc(1).send({data, 2049}));
    assert(bsp.cdc(1).printf("%0200d", 1));
    assert(usb_tx_size == 127);
    static int usb_callbacks = 0;
    bsp.cdc(1).add_rx_callback([](Bytes bytes) { assert(bytes.size == 3); ++usb_callbacks; });
    bsp.cdc(1).add_rx_callback([](Bytes) { ++usb_callbacks; });
    bsp_cdc_receive(data, 3);
    assert(usb_callbacks == 2);
}

static void test_pwm_timer() {
    assert(bsp.pwm(4, 3) && !bsp.pwm(4, 1) && !bsp.pwm(2, 1));
    assert(!bsp.pwm(4, 1).set_duty(1));
    assert(bsp.pwm(1, 1).set_duty(1) && reg1.CCR[0] == reg1.ARR + 1);
    auto pwm = bsp.pwm(4, 3);
    assert(pwm.set_frequency(2000));
    assert(84000000.0 / (reg4.PSC + 1) / (reg4.ARR + 1) == 2000);
    assert(pwm.set_duty(0.5f) && reg4.CCR[2] == (reg4.ARR + 1) / 2);
    assert(pwm.set_duty(-1) && reg4.CCR[2] == 0);
    const auto period = reg4.ARR;
    assert(!pwm.set_frequency(0) && !pwm.set_frequency(1e-10f));
    assert(!pwm.set_frequency(std::numeric_limits<float>::infinity()) && reg4.ARR == period);
    assert(!pwm.set_duty(std::numeric_limits<float>::quiet_NaN()));
    bsp.pwm(5, 1).set_duty(0.25f);
    bsp.pwm(5, 2).set_duty(0.75f);
    bsp.pwm(5, 3).set_frequency(1000);
    assert(reg5.CCR[0] == (reg5.ARR + 1) / 4 && reg5.CCR[1] == (reg5.ARR + 1) * 3 / 4);
    Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
    Rgb invalid({});
    Rgb partial({.red = Pwm(5, 3)});
    assert(rgb && !invalid && !partial);
    assert(!invalid.set_rgb(1, 1, 1) && !partial.set_rgb(1, 1, 1));
    assert(rgb.set_hsv(0, 1, 1));
    assert(reg5.CCR[2] == reg5.ARR + 1 && reg5.CCR[0] == 0 && reg5.CCR[1] == 0);

    Timer first(0), second(0);
    DWT->CYCCNT = 100;
    assert(first.get_dt() == 0);
    DWT->CYCCNT += SystemCoreClock / 100;
    assert(std::fabs(first.get_dt() - 0.01f) < 1e-6f && second.get_dt() == 0);
    first.reset();
    DWT->CYCCNT += SystemCoreClock / 50;
    assert(first.poll_timeout(0.01f) && !first.poll_timeout(0.01f));
    DWT->CYCCNT = UINT32_MAX - 99;
    first.reset();
    DWT->CYCCNT = 100;
    assert(std::fabs(first.update_dt() - 200.0f / SystemCoreClock) < 1e-9f);
    Timer clamped(0.001f, 0.01f);
    assert(clamped.get_dt() == 0.001f);
    DWT->CYCCNT += SystemCoreClock;
    assert(clamped.get_dt() == 0.01f && clamped.update_dt() == 0.01f);
    assert(clamped.get_dt() == 0.001f);
    auto copied = clamped;
    assert(copied == clamped);
    DWT->CYCCNT += 1;
    copied.reset();
    assert(copied != clamped);
}

static void test_metrics() {
    CallbackTiming timing;
    volatile float usage = 0;
    // 每毫秒一次、每次 20 us，真实负载 2%。
    for (unsigned ms = 1; ms <= 10000; ++ms)
        timing.record(20, ms, usage);
    assert(std::fabs(usage - 0.02f) < 1e-6f);

    CallbackTiming burst;
    volatile float burst_usage = 0;
    for (unsigned ms = 5; ms <= 10000; ms += 5) {
        for (int frame = 0; frame < 5; ++frame)
            burst.record(20, ms, burst_usage);
    }
    assert(std::fabs(burst_usage - 0.02f) < 1e-6f);

    // 原地保留最后结果；下一次接收将空闲时间计入区间。
    const float previous = usage;
    timing.record(20, 20000, usage);
    assert(usage > 0 && usage < previous * 0.05f);
    for (unsigned ms = 20001; ms <= 30000; ++ms)
        timing.record(40, ms, usage);
    assert(std::fabs(usage - 0.04f) < 1e-6f);

    CallbackTiming wrap{.window_start_ms = UINT32_MAX - 59};
    volatile float wrapped_usage = 0;
    wrap.record(2000, 40, wrapped_usage);
    assert(std::fabs(wrapped_usage - 0.004f) < 1e-7f);
}

static void test_spi() {
    Spi accel(1, 1);
    auto gyro = bsp.spi(1, 2);
    assert(accel && gyro && !Spi(2, 1) && !Spi(1, 3) && !Spi());
    const std::uint8_t tx[] = {0x80, 0x55, 0x55};
    std::uint8_t rx[10]{};
    const auto transfer = SpiTransfer{.tx = tx, .rx = rx, .size = 3};
    assert(accel.transfer(transfer));
    assert(spi_packets.back().pin == CS1_ACCEL_Pin && spi_packets.back().timeout == 10);
    assert(spi_edges.back().second == GPIO_PIN_SET && irq_mask == 0);
    const auto edges = spi_edges.size();
    assert(!Spi(9, 9).transfer(transfer));
    assert(!accel.transfer({.tx = tx, .rx = rx, .size = 65536}));
    assert(!accel.transfer({}));
    hspi1.State = HAL_SPI_STATE_BUSY;
    assert(!accel.transfer(transfer) && spi_edges.size() == edges);
    hspi1.State = HAL_SPI_STATE_READY;
    spi_result = HAL_ERROR;
    assert(!gyro.transfer(transfer));
    assert(spi_edges.back() == std::make_pair(CS1_GYRO_Pin, GPIO_PIN_SET));
    spi_result = HAL_OK;

    bmi088_bind(accel, gyro);
    bmi088_accel_write_reg(0x40, 0x12);
    assert((spi_packets.back().tx == std::vector<std::uint8_t>{0x40, 0x12}));
    assert(bmi088_accel_read_reg(0x40) == 2);
    assert((spi_packets.back().tx == std::vector<std::uint8_t>{0xc0, 0x55, 0x55}));
    assert(bmi088_gyro_read_reg(0) == 1);
    assert((spi_packets.back().tx == std::vector<std::uint8_t>{0x80, 0x55}));
    bmi088_accel_read_regs(0x12, rx, 6);
    assert((spi_packets.back().tx == std::vector<std::uint8_t>{0x92, 0x92, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}));
    assert(rx[0] == 2 && rx[5] == 7);
    bmi088_gyro_read_regs(0, rx, 8);
    assert(spi_packets.back().tx.size() == 9 && rx[0] == 1 && rx[7] == 8);
    spi_result = HAL_ERROR;
    bmi088_gyro_read_regs(0, rx, 8);
    assert(rx[0] == 0 && rx[7] == 0 && spi_edges.back().second == GPIO_PIN_SET);
    spi_result = HAL_OK;
}

int main() {
    DwtRegisters cycles;
    DebugRegisters debug;
    DWT = &cycles;
    CoreDebug = &debug;
    DMA_HandleTypeDef dma3, dma6;
    huart3.hdmarx = &dma3;
    huart6.hdmarx = &dma6;
    huart3.Init.Mode = UART_MODE_RX;
    call_bsp_from_c();
    test_can();
    test_uart_usb();
    test_pwm_timer();
    test_metrics();
    test_spi();
    std::puts("bsp: all checks passed");
}
