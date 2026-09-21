#include "bsp.hpp"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using rmpp::board;
auto global_can = board.can(1);
auto global_timer = board.timer();
int early_frames = 0;
const bool registered_early = global_can.add_rx_callback([](const rmpp::CanFrame&) { ++early_frames; });
extern "C" void call_bsp_from_c(void);

static void test_can() {
    assert(registered_early && board.can(1) && board.can(2));
    assert(!board.can(0) && !board.can(3) && !board.can(-1));
    rmpp::CanFrame frame{.id = 0x123, .data = {1, 2, 3}, .size = 3};
    assert(!board.can(99).send(frame));
    assert(!board.can(99).add_rx_callback([](const auto&) {}));
    assert(board.can(99).cpu_usage() == 0 && board.can(99).bus_off_count() == 0);
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
            board.can(1).add_rx_callback([this](const rmpp::CanFrame& f) { assert(f.size == 2); order.push_back(1); });
            board.can(1).add_rx_callback([this](const rmpp::CanFrame&) { order.push_back(2); });
        }
    };
    static Receiver receiver;
    irq_mask = 1;
    receiver.subscribe();
    assert(irq_mask == 1);
    irq_mask = 0;
    static int can2_frames = 0;
    board.can(2).add_rx_callback([](const rmpp::CanFrame&) { ++can2_frames; });
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
    assert(board.can(2).bus_off_count() == 1 && board.can(1).bus_off_count() == 0);
}

static void test_uart_usb() {
    assert(board.uart(1) && board.uart(3) && board.uart(6) && !board.uart(2));
    std::uint8_t data[] = {10, 20, 30};
    assert(!board.uart(2).send({data, 3}) && !board.uart(3).send({data, 3}));
    assert(board.uart(6).send({data, 3}));
    data[0] = 99;
    assert(huart6.tx != data && huart6.tx[0] == 10);
    assert(!board.uart(6).send({data, 3}) && huart6.tx[0] == 10);
    huart6.gState = HAL_UART_STATE_READY;
    assert(!board.uart(6).send({data, 257}));
    static std::vector<unsigned> sizes;
    board.uart(3).add_rx_callback([](rmpp::Bytes bytes) { sizes.push_back(bytes.size); });
    board.uart(3).add_rx_callback([](rmpp::Bytes bytes) { sizes.push_back(bytes.size + 1); });
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
    assert(board.uart(3).error_count() == 1 && huart3.rearms == rearms + 3);
    assert(huart1.rearms == 1);

    assert(board.cdc(1) && !board.cdc(2));
    assert(!board.cdc(1).send({data, 3}));
    static USBD_CDC_HandleTypeDef cdc;
    hUsbDeviceFS = {USBD_STATE_CONFIGURED, &cdc};
    assert(board.cdc(1).send({data, 3}));
    data[0] = 77;
    assert(usb_tx != data && usb_tx[0] == 99);
    assert(!board.cdc(1).send({data, 3}) && usb_tx[0] == 99);
    cdc.TxState = 0;
    assert(!board.cdc(1).send({data, 2049}));
    assert(board.cdc(1).printf("%0200d", 1));
    assert(usb_tx_size == 127);
    static int usb_callbacks = 0;
    board.cdc(1).add_rx_callback([](rmpp::Bytes bytes) { assert(bytes.size == 3); ++usb_callbacks; });
    board.cdc(1).add_rx_callback([](rmpp::Bytes) { ++usb_callbacks; });
    bsp_cdc_receive(data, 3);
    assert(usb_callbacks == 2);
}

static void test_pwm_timer() {
    assert(board.pwm(4, 3) && !board.pwm(4, 1) && !board.pwm(2, 1));
    assert(!board.pwm(4, 1).set_duty(1));
    assert(board.pwm(1, 1).set_duty(1) && reg1.CCR[0] == reg1.ARR + 1);
    auto pwm = board.pwm(4, 3);
    assert(pwm.set_frequency(2000));
    assert(84000000.0 / (reg4.PSC + 1) / (reg4.ARR + 1) == 2000);
    assert(pwm.set_duty(0.5f) && reg4.CCR[2] == (reg4.ARR + 1) / 2);
    assert(pwm.set_duty(-1) && reg4.CCR[2] == 0);
    const auto period = reg4.ARR;
    assert(!pwm.set_frequency(0) && !pwm.set_frequency(1e-10f));
    assert(!pwm.set_frequency(std::numeric_limits<float>::infinity()) && reg4.ARR == period);
    assert(!pwm.set_duty(std::numeric_limits<float>::quiet_NaN()));
    board.pwm(5, 1).set_duty(0.25f);
    board.pwm(5, 2).set_duty(0.75f);
    board.pwm(5, 3).set_frequency(1000);
    assert(reg5.CCR[0] == (reg5.ARR + 1) / 4 && reg5.CCR[1] == (reg5.ARR + 1) * 3 / 4);
    assert(board.led(1) && !board.led(2));
    assert(!board.led(2).set_rgb(1, 1, 1));
    assert(board.led(1).set_hsv(0, 1, 1));
    assert(reg5.CCR[2] == reg5.ARR + 1 && reg5.CCR[0] == 0 && reg5.CCR[1] == 0);

    rmpp::Timer first(0), second(0);
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
    rmpp::Timer clamped(0.001f, 0.01f);
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
    std::puts("board: all checks passed");
}
