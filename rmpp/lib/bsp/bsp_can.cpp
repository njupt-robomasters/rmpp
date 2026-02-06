#include "bsp_can.hpp"
#include "can.h"

using namespace BSP;

std::vector<CAN::CallbackFunc>* CAN::callbacks;
Dwt CAN::dwt;
UnitFloat<pct> CAN::cpu_usage;
uint32_t CAN::bus_off_cnt = 0;

void CAN::Init() {
    CAN_FilterTypeDef can_filter_st;

    // CAN1
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF); // 开启中断

    // CAN2
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF); // 开启中断
}

void CAN::TransmitStd(const uint8_t port, const uint32_t id, uint8_t data[8], const uint8_t dlc) {
    CAN_TxHeaderTypeDef header;
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;

    uint32_t send_mail_box;
    if (port == 1) {
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {} // 等待发送邮箱有空余位置
        HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
    } else if (port == 2) {
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {} // 等待发送邮箱有空余位置
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &send_mail_box);
    }
}

void CAN::TransmitExt(const uint8_t port, const uint32_t id, uint8_t data[8], const uint8_t dlc) {
    CAN_TxHeaderTypeDef header;
    header.ExtId = id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;

    uint32_t send_mail_box;
    if (port == 1) {
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {} // 等待发送邮箱有空余位置
        HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
    } else if (port == 2) {
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {} // 等待发送邮箱有空余位置
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &send_mail_box);
    }
}

void CAN::RegisterCallback(const CallbackFunc& callback) {
    if (!callbacks) {
        callbacks = new std::vector<CallbackFunc>;
    }
    callbacks->push_back(callback);
}

void CAN::InvokeCallback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    const UnitFloat interval_time = dwt.UpdateDT();
    if (callbacks) {
        for (const auto& callback : *callbacks) {
            callback(port, id, data, dlc);
        }
    }
    const UnitFloat running_time = dwt.GetDT();
    cpu_usage = running_time / interval_time;
}

/********************* 以下为HAL库回调函数 ********************/

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);

    uint32_t id = 0;
    if (header.IDE == CAN_ID_STD) {
        id = header.StdId;
    } else if (header.IDE == CAN_ID_EXT) {
        id = header.ExtId;
    }

    if (hcan->Instance == hcan1.Instance) {
        CAN::InvokeCallback(1, id, data, header.DLC);
    } else if (hcan->Instance == hcan2.Instance) {
        CAN::InvokeCallback(2, id, data, header.DLC);
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    const uint32_t error_code = hcan->ErrorCode;
    if (error_code & HAL_CAN_ERROR_BOF) {
        CAN::bus_off_cnt++;
    }
}