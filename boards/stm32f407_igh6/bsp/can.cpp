#include "bsp/can.hpp"
#include "can.h"
#include "peripherals.hpp"
#include <utility>
#include <vector>

struct Can::State {
    int id;
    CAN_HandleTypeDef* handle;
    std::vector<RxCallback> callbacks;

    struct Metrics {
        float last_callback_us = 0;
        float cpu_usage = 0;
        std::uint32_t bus_off_count = 0;
    };

    volatile Metrics metrics{};
    CallbackTiming timing;
};

Can::Can(int id) : id_(id) {
    static constinit State states[] = {
        {.id = 1, .handle = &hcan1},
        {.id = 2, .handle = &hcan2},
    };
    for (auto& state : states) {
        if (state.id == id_) {
            state_ = &state;
            break;
        }
    }
}

bool Can::send(const CanFrame& frame) const {
    if (!state_ || frame.size > frame.data.size() || frame.id > (frame.extended ? 0x1fffffffu : 0x7ffu))
        return false;
    CAN_TxHeaderTypeDef header{
        .StdId = frame.extended ? 0 : frame.id,
        .ExtId = frame.extended ? frame.id : 0,
        .IDE = frame.extended ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = frame.remote ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = frame.size,
    };
    std::uint32_t mailbox;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    const bool success = HAL_CAN_AddTxMessage(state_->handle, &header, frame.data.data(), &mailbox) == HAL_OK;
    __set_PRIMASK(primask);
    return success;
}

bool Can::add_rx_callback(RxCallback callback) const {
    if (!state_ || !callback)
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    state_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
    return true;
}

void can_init() {
    for (int id : {1, 2}) {
        auto* state = Can(id).state_;
        CAN_FilterTypeDef filter{
            .FilterFIFOAssignment = id == 1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
            .FilterBank = id == 1 ? 0u : 14u,
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterScale = CAN_FILTERSCALE_32BIT,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
        };
        state->timing.window_start_ms = HAL_GetTick();
        HAL_CAN_ConfigFilter(state->handle, &filter);
        HAL_CAN_Start(state->handle);
        HAL_CAN_ActivateNotification(state->handle,
                                     (id == 1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING) | CAN_IT_ERROR | CAN_IT_BUSOFF);
    }
}

void can_receive(int id, std::uint32_t fifo) {
    auto* state = Can(id).state_;
    if (!state)
        return;
    while (HAL_CAN_GetRxFifoFillLevel(state->handle, fifo)) {
        CAN_RxHeaderTypeDef header{};
        std::array<std::uint8_t, 8> data{};
        if (HAL_CAN_GetRxMessage(state->handle, fifo, &header, data.data()) != HAL_OK)
            break;
        if (header.DLC > data.size())
            continue;

        const CanFrame frame{
            .id = header.IDE == CAN_ID_EXT ? header.ExtId : header.StdId,
            .data = data,
            .size = static_cast<std::uint8_t>(header.DLC),
            .extended = header.IDE == CAN_ID_EXT,
            .remote = header.RTR == CAN_RTR_REMOTE,
        };
        const auto begin = DWT->CYCCNT;
        for (const auto& callback : state->callbacks)
            callback(frame);
        const float duration_us = static_cast<std::uint32_t>(DWT->CYCCNT - begin) * (1000000.0f / SystemCoreClock);
        state->metrics.last_callback_us = duration_us;
        state->timing.record(duration_us, HAL_GetTick(), state->metrics.cpu_usage);
    }
}

void can_error(int id) {
    auto* state = Can(id).state_;
    if (state && (state->handle->ErrorCode & HAL_CAN_ERROR_BOF))
        state->metrics.bus_off_count = state->metrics.bus_off_count + 1;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle) {
    can_receive(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                        : 0,
                CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* handle) {
    can_receive(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                        : 0,
                CAN_RX_FIFO1);
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle) {
    can_error(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                      : 0);
}
