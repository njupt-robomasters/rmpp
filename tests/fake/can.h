#pragma once
#include "stm32f4xx.h"
#include <array>
#include <cstring>
#include <deque>

inline constexpr unsigned CAN_ID_STD = 0, CAN_ID_EXT = 4, CAN_RTR_DATA = 0, CAN_RTR_REMOTE = 2;
inline constexpr unsigned CAN_RX_FIFO0 = 0, CAN_RX_FIFO1 = 1, ENABLE = 1;
inline constexpr unsigned CAN_FILTERMODE_IDMASK = 0, CAN_FILTERSCALE_32BIT = 1;
inline constexpr unsigned CAN_IT_RX_FIFO0_MSG_PENDING = 1, CAN_IT_RX_FIFO1_MSG_PENDING = 2;
inline constexpr unsigned CAN_IT_ERROR = 4, CAN_IT_BUSOFF = 8, HAL_CAN_ERROR_BOF = 8;
struct CAN_TxHeaderTypeDef {
    std::uint32_t StdId{}, ExtId{}, IDE{}, RTR{}, DLC{};
};
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
struct CAN_FilterTypeDef {
    std::uint32_t FilterActivation{}, FilterMode{}, FilterScale{}, FilterBank{}, FilterFIFOAssignment{}, SlaveStartFilterBank{};
};
struct CanPacket {
    CAN_RxHeaderTypeDef header;
    std::array<std::uint8_t, 8> data;
};
struct CAN_HandleTypeDef {
    std::uint32_t ErrorCode = 0;
    int available = 3, submissions = 0;
    bool active = false;
    CAN_TxHeaderTypeDef sent_header{};
    std::array<std::uint8_t, 8> sent_data{};
    std::deque<CanPacket> received;
};
inline CAN_HandleTypeDef hcan1, hcan2;
inline HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h) {
    h->active = true;
    return HAL_OK;
}
inline HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, std::uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* h, CAN_TxHeaderTypeDef* header, const std::uint8_t* data, std::uint32_t*) {
    ++h->submissions;
    if (!h->active || !h->available)
        return HAL_ERROR;
    --h->available;
    h->sent_header = *header;
    std::memcpy(h->sent_data.data(), data, 8);
    return HAL_OK;
}
inline unsigned HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* h, unsigned) { return h->received.size(); }
inline HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* h, unsigned, CAN_RxHeaderTypeDef* header, std::uint8_t* data) {
    if (h->received.empty())
        return HAL_ERROR;
    *header = h->received.front().header;
    std::memcpy(data, h->received.front().data.data(), 8);
    h->received.pop_front();
    return HAL_OK;
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*);
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef*);
