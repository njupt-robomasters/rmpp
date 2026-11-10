/*
 * CubeMX 配置：Connectivity → CAN1 / CAN2，APB1 = 42 MHz 时为 1 Mbps
 *
 * 两路通用参数
 * | 页面               | 参数                             | 设置               |
 * | ------------------ | -------------------------------- | ------------------ |
 * | Mode               | Activated                        | 勾选               |
 * | Parameter Settings | Operating Mode                   | Normal             |
 * | Parameter Settings | Prescaler (for Time Quantum)     | 3                  |
 * | Parameter Settings | Time Quanta in Bit Segment 1 / 2 | 10 Times / 3 Times |
 * | Parameter Settings | ReSynchronization Jump Width     | 1 Time             |
 * | Parameter Settings | Automatic Bus-Off Management     | Enable             |
 * | Parameter Settings | Automatic Retransmission         | Enable             |
 * | Parameter Settings | Transmit Fifo Priority           | Enable             |
 *
 * 引脚与 NVIC Settings
 * | 外设 | Pinout                       | 勾选中断           | Preemption / Sub Priority |
 * | ---- | ---------------------------- | ------------------ | ------------------------- |
 * | CAN1 | PD0 = CAN1_RX，PD1 = CAN1_TX | CAN1 RX0、CAN1 SCE | 0 / 0                     |
 * | CAN2 | PB5 = CAN2_RX，PB6 = CAN2_TX | CAN2 RX1、CAN2 SCE | 0 / 0                     |
 *
 * 过滤器及接收通知由 can_init() 设置，时钟、GPIO 和 IRQHandler 由 CubeMX 生成
 * 发送使用 3 个邮箱，按请求顺序发送；自动重发开启
 */

#include "bsp/can.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "can.h"
#include <vector>

struct CanContext {
    int id;
    CAN_HandleTypeDef* hcan;
    uint32_t fifo; // CAN1 用 FIFO0，CAN2 用 FIFO1
    std::vector<Can::RxCallback> callbacks;
    Monitor monitor;
    uint32_t bus_off_count = 0; // 总线关闭（bus-off）次数
};

static constinit CanContext contexts[] = {
    {.id = 1, .hcan = &hcan1, .fifo = CAN_RX_FIFO0},
    {.id = 2, .hcan = &hcan2, .fifo = CAN_RX_FIFO1},
};

static CanContext* find(const int id) {
    for (auto& context : contexts)
        if (context.id == id)
            return &context;
    return nullptr;
}

static CanContext* find(const CAN_HandleTypeDef* hcan) {
    for (auto& context : contexts)
        if (context.hcan == hcan)
            return &context;
    return nullptr;
}

void can_init() {
    for (const auto& context : contexts) {
        // 全部帧通过；CAN2 从过滤器 14 开始
        const bool is_can1 = context.id == 1;
        CAN_FilterTypeDef filter{
            .FilterFIFOAssignment = context.fifo,
            .FilterBank = is_can1 ? 0u : 14u,
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
        };
        HAL_CAN_ConfigFilter(context.hcan, &filter);
        HAL_CAN_Start(context.hcan);

        const uint32_t rx_it = is_can1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;
        HAL_CAN_ActivateNotification(context.hcan, rx_it | CAN_IT_ERROR | CAN_IT_BUSOFF);
    }
}

bool Can::can_send() const {
    const auto* context = find(id_);
    return context && HAL_CAN_GetTxMailboxesFreeLevel(context->hcan) != 0;
}

void Can::send(const CanFrame& frame) const {
    const auto* context = find(id_);
    if (!context)
        return;

    const CAN_TxHeaderTypeDef header{
        .StdId = frame.is_extend ? 0 : frame.id,
        .ExtId = frame.is_extend ? frame.id : 0,
        .IDE = frame.is_extend ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = frame.is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = frame.size,
    };
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(context->hcan, &header, frame.data, &mailbox);
}

void Can::add_rx_callback(RxCallback callback) const {
    auto* context = find(id_);
    if (!context)
        return;

    const IrqLock lock;
    context->callbacks.push_back(std::move(callback));
}

static void can_receive(CAN_HandleTypeDef* hcan) {
    auto* context = find(hcan);
    if (!context)
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, context->fifo)) {
        CanFrame frame;

        CAN_RxHeaderTypeDef header{};
        if (HAL_CAN_GetRxMessage(hcan, context->fifo, &header, frame.data) != HAL_OK)
            break;

        frame.id = header.IDE == CAN_ID_STD ? header.StdId : header.ExtId;
        frame.is_extend = header.IDE == CAN_ID_EXT;
        frame.is_remote = header.RTR == CAN_RTR_REMOTE;
        frame.size = static_cast<uint8_t>(header.DLC);

        context->monitor.begin();
        for (const auto& callback : context->callbacks)
            callback(frame);
        context->monitor.end();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    can_receive(hcan);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    can_receive(hcan);
}

// bus-off 由硬件自动恢复（Automatic Bus-Off Management），这里只计数
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    auto* context = find(hcan);
    if (context && (hcan->ErrorCode & HAL_CAN_ERROR_BOF))
        context->bus_off_count++;
}
