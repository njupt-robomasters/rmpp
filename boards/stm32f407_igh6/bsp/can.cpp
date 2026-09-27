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
 * | Parameter Settings | Automatic Retransmission         | Disable            |
 *
 * 引脚与 NVIC Settings
 * | 外设 | Pinout                       | 勾选中断           | Preemption / Sub Priority |
 * | ---- | ---------------------------- | ------------------ | ------------------------- |
 * | CAN1 | PD0 = CAN1_RX，PD1 = CAN1_TX | CAN1 RX0、CAN1 SCE | 0 / 0                     |
 * | CAN2 | PB5 = CAN2_RX，PB6 = CAN2_TX | CAN2 RX1、CAN2 SCE | 0 / 0                     |
 *
 * 过滤器及接收通知由 can_init() 设置，时钟、GPIO 和 IRQHandler 由 CubeMX 生成
 */

#include "bsp/can.hpp"
#include "bsp/monitor.hpp"
#include "can.h"

struct Can::Context {
    int id;
    CAN_HandleTypeDef* hcan;
    std::vector<RxCallback> callbacks;
    Monitor monitor;
    uint32_t bus_off_count = 0; // 总线关闭（bus-off）次数
};

void can_init() {
    for (const int id : {1, 2}) {
        auto* context = Can(id).context_;
        CAN_FilterTypeDef filter{
            .FilterFIFOAssignment = id == 1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
            .FilterBank = id == 1 ? 0u : 14u,
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
        };
        HAL_CAN_ConfigFilter(context->hcan, &filter);
        HAL_CAN_Start(context->hcan);
        HAL_CAN_ActivateNotification(context->hcan,
                                     (id == 1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING) | CAN_IT_ERROR | CAN_IT_BUSOFF);
    }
}

Can::Can(const int id) {
    static constinit Context contexts[] = {
        {.id = 1, .hcan = &hcan1},
        {.id = 2, .hcan = &hcan2},
    };
    if (id == 1)
        context_ = &contexts[0];
    else if (id == 2)
        context_ = &contexts[1];
}

bool Can::send(const CanFrame& frame) const {
    if (!context_)
        return false;

    const CAN_TxHeaderTypeDef header{
        .StdId = frame.is_extend ? 0 : frame.id,
        .ExtId = frame.is_extend ? frame.id : 0,
        .IDE = frame.is_extend ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = frame.is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = frame.size,
    };
    uint32_t mailbox;
    const bool success = HAL_CAN_AddTxMessage(context_->hcan, &header, frame.data, &mailbox) == HAL_OK;

    return success;
}

bool Can::add_rx_callback(RxCallback callback) const {
    if (!context_)
        return false;

    const auto primask = __get_PRIMASK();
    __disable_irq();
    context_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);

    return true;
}

static void can_receive(CAN_HandleTypeDef* hcan, const uint32_t fifo) {
    Can::Context* context;
    if (hcan == &hcan1)
        context = Can(1).context_;
    else if (hcan == &hcan2)
        context = Can(2).context_;
    else
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo)) {
        CanFrame frame;

        CAN_RxHeaderTypeDef header{};
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, frame.data) != HAL_OK)
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
    can_receive(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    can_receive(hcan, CAN_RX_FIFO1);
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    Can::Context* context;
    if (hcan == &hcan1)
        context = Can(1).context_;
    else if (hcan == &hcan2)
        context = Can(2).context_;
    else
        return;

    if (hcan->ErrorCode & HAL_CAN_ERROR_BOF)
        context->bus_off_count += 1;
}
