#include "board/can.hpp"
#include "board/timer.hpp"
#include "can.h"
#include "irq_lock.hpp"
#include "peripherals.hpp"
#include <utility>
#include <vector>

namespace rmpp {
struct Can::State {
    CAN_HandleTypeDef* handle;
    std::vector<RxCallback> callbacks;
    Timer timer{0};
    float cpu_usage = 0;
    std::uint32_t bus_off_count = 0;
};

Can::State* Can::find(int id) {
    switch (id) {
    case 1: {
        static constinit State state{&hcan1};
        return &state;
    }
    case 2: {
        static constinit State state{&hcan2};
        return &state;
    }
    default:
        return nullptr;
    }
}

bool Can::send(const CanFrame& frame) const {
    if (!state_ || frame.size > frame.data.size() || frame.id > (frame.extended ? 0x1fffffffu : 0x7ffu))
        return false;
    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.extended ? 0 : frame.id;
    header.ExtId = frame.extended ? frame.id : 0;
    header.IDE = frame.extended ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = frame.remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header.DLC = frame.size;
    std::uint32_t mailbox;
    IrqLock lock;
    return HAL_CAN_AddTxMessage(state_->handle, &header, frame.data.data(), &mailbox) == HAL_OK;
}

bool Can::add_rx_callback(RxCallback callback) const {
    if (!state_ || !callback)
        return false;
    IrqLock lock;
    state_->callbacks.push_back(std::move(callback));
    return true;
}

float Can::cpu_usage() const { return state_ ? state_->cpu_usage : 0; }
std::uint32_t Can::bus_off_count() const { return state_ ? state_->bus_off_count : 0; }

void can_init() {
    for (int id : {1, 2}) {
        auto* state = Can::find(id);
        CAN_FilterTypeDef filter{};
        filter.FilterActivation = ENABLE;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterBank = id == 1 ? 0 : 14;
        filter.FilterFIFOAssignment = id == 1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
        filter.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(state->handle, &filter);
        HAL_CAN_Start(state->handle);
        HAL_CAN_ActivateNotification(state->handle,
                                     (id == 1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING) | CAN_IT_ERROR | CAN_IT_BUSOFF);
    }
}

void can_receive(int id, std::uint32_t fifo) {
    auto* state = Can::find(id);
    if (!state)
        return;
    while (HAL_CAN_GetRxFifoFillLevel(state->handle, fifo)) {
        CAN_RxHeaderTypeDef header{};
        CanFrame frame;
        if (HAL_CAN_GetRxMessage(state->handle, fifo, &header, frame.data.data()) != HAL_OK)
            break;
        frame.extended = header.IDE == CAN_ID_EXT;
        frame.id = frame.extended ? header.ExtId : header.StdId;
        frame.remote = header.RTR == CAN_RTR_REMOTE;
        frame.size = header.DLC;
        const float interval = state->timer.update_dt();
        for (const auto& callback : state->callbacks)
            callback(frame);
        state->cpu_usage = interval > 0 ? state->timer.get_dt() / interval : 0;
    }
}

void can_error(int id) {
    auto* state = Can::find(id);
    if (state && (state->handle->ErrorCode & HAL_CAN_ERROR_BOF))
        ++state->bus_off_count;
}
} // namespace rmpp

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle) {
    rmpp::can_receive(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                              : 0,
                      CAN_RX_FIFO0);
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* handle) {
    rmpp::can_receive(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                              : 0,
                      CAN_RX_FIFO1);
}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle) {
    rmpp::can_error(handle == &hcan1 ? 1 : handle == &hcan2 ? 2
                                                            : 0);
}
