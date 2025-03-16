#include "bsp_can.h"
#include "can.h"
#include "task_chassis.h"
#include "task_gimbal.h"

void BSP_CAN_Init() {
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
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // CAN2
    // can_filter_st.SlaveStartFilterBank = 14;
    // can_filter_st.FilterBank = 14;
    // HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    // HAL_CAN_Start(&hcan2);
    // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void BSP_CAN_Transmit(const uint32_t id, uint8_t data[8], const uint8_t dlc) {
    CAN_TxHeaderTypeDef header;
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;

    uint32_t send_mail_box;

    HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
}

void BSP_CAN_Rx_Callback(uint32_t id, uint8_t data[8]) {
    task_chassis_callback(id, data);
    task_gimbal_callback(id, data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
    BSP_CAN_Rx_Callback(header.StdId, data);
}
