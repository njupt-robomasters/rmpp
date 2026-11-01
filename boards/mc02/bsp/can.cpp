/*
 * FDCAN1/2/3：CubeMX 配置为经典帧 1 Mbps（内核时钟 PLL2Q 125 MHz，Prescaler 5，Seg1 19，Seg2 5）
 * | 外设   | RX / TX     | RAM 偏移 |
 * | ------ | ----------- | -------- |
 * | FDCAN1 | PD0 / PD1   | 0x000    |
 * | FDCAN2 | PB5 / PB6   | 0x358    |
 * | FDCAN3 | PD12 / PD13 | 生成值   |
 *
 * 全局过滤把标准帧和扩展帧都送入 RX FIFO0；发送使用 32 级 TX FIFO；IT0 中断由 CubeMX 开启
 * bus-off 后硬件置位 CCCR.INIT，这里计数并清除，经 129 × 11 个隐性位后自动重新入网
 */

#include "bsp/can.hpp"
#include "bsp/monitor.hpp"
#include "fdcan.h"
#include <algorithm>
#include <cstring>
#include <vector>

struct CanContext {
    int id;
    FDCAN_HandleTypeDef* hfdcan;
    std::vector<Can::RxCallback> callbacks;
    Monitor monitor;
    uint32_t bus_off_count = 0; // 总线关闭（bus-off）次数
};

static constinit CanContext contexts[] = {
    {.id = 1, .hfdcan = &hfdcan1},
    {.id = 2, .hfdcan = &hfdcan2},
    {.id = 3, .hfdcan = &hfdcan3},
};

static CanContext* find(const int id) {
    for (auto& context : contexts)
        if (context.id == id)
            return &context;
    return nullptr;
}

static CanContext* find(const FDCAN_HandleTypeDef* hfdcan) {
    for (auto& context : contexts)
        if (context.hfdcan == hfdcan)
            return &context;
    return nullptr;
}

void can_init() {
    for (const auto& context : contexts) {
        HAL_FDCAN_ConfigGlobalFilter(context.hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ActivateNotification(context.hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
        HAL_FDCAN_Start(context.hfdcan);
    }
}

bool Can::send(const CanFrame& frame) const {
    const auto* context = find(id_);
    if (!context)
        return false;

    // 经典帧 DLC 0–8 与 FDCAN_DLC_BYTES_0–8 数值相同
    const FDCAN_TxHeaderTypeDef header{
        .Identifier = frame.id,
        .IdType = frame.is_extend ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType = frame.is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
        .DataLength = std::min<uint32_t>(frame.size, 8),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0,
    };
    return HAL_FDCAN_AddMessageToTxFifoQ(context->hfdcan, &header, frame.data) == HAL_OK;
}

void Can::add_rx_callback(RxCallback callback) const {
    auto* context = find(id_);
    if (!context)
        return;

    const auto primask = __get_PRIMASK();
    __disable_irq();
    context->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t) {
    auto* context = find(hfdcan);
    if (!context)
        return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
        // 经典帧 DLC 9–15 也表示 8 字节，但 HAL 按 FD 长度表拷贝，接收区按 64 字节准备
        FDCAN_RxHeaderTypeDef header{};
        uint8_t data[64]{};
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
            break;

        CanFrame frame;
        frame.id = header.Identifier;
        frame.is_extend = header.IdType == FDCAN_EXTENDED_ID;
        frame.is_remote = header.RxFrameType == FDCAN_REMOTE_FRAME;
        frame.size = static_cast<uint8_t>(std::min<uint32_t>(header.DataLength, 8));
        std::memcpy(frame.data, data, sizeof(frame.data));

        context->monitor.begin();
        for (const auto& callback : context->callbacks)
            callback(frame);
        context->monitor.end();
    }
}

// 进入 bus-off 时计数并清 INIT，硬件完成恢复序列后自动重新入网
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, const uint32_t ErrorStatusITs) {
    auto* context = find(hfdcan);
    if (!context || !(ErrorStatusITs & FDCAN_IT_BUS_OFF) || !(hfdcan->Instance->PSR & FDCAN_PSR_BO))
        return;

    context->bus_off_count++;
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}
