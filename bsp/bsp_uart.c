#include "bsp_uart.h"
#include "usart.h"

#include "task_protocol.h"

#define rc_rxbuf_size 50
#define referee_rxbuf_size 127
#define referee_video_rxbuf_size 127

static uint8_t rc_rxbuf[rc_rxbuf_size];
static uint8_t referee_rxbuf[referee_rxbuf_size];
static uint8_t referee_video_rxbuf[referee_video_rxbuf_size];
static volatile uint8_t is_referee_video_tx_idle = 1;

void BSP_UART_Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_video_rxbuf, referee_video_rxbuf_size);
}

void BSP_UART_RC_Rx_Callback(const uint8_t *data, const uint16_t size) {
    task_protocol_rc_callback(data, size);
}

void BSP_UART_Referee_Rx_Callback(const uint8_t *data, const uint16_t size) {
    task_protocol_referee_callback(data, size);
}

void BSP_UART_Referee_Video_Rx_Callback(const uint8_t *data, const uint16_t size) {
    task_protocol_referee_callback(data, size);
}

void BSP_UART_Referee_Video_Transmit(const uint8_t *data, const uint16_t size) {
    is_referee_video_tx_idle = 0;
    HAL_UART_Transmit_DMA(&huart6, data, size);
}

uint8_t BSP_UART_Referee_Video_CheckIdle() {
    return is_referee_video_tx_idle;
}


/********************* 以下为HAL库回调函数 ********************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    // 遥控器
    if (huart->Instance == USART3) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart3);
            BSP_UART_RC_Rx_Callback(rc_rxbuf, Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
        }
    }

    // 裁判系统学生串口（来自裁判系统电管）
    if (huart->Instance == USART6) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart6);
            BSP_UART_Referee_Rx_Callback(referee_rxbuf, Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
        }
    }

    // 裁判系统图传串口（来自图传发送端）
    if (huart->Instance == USART1) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart1);
            BSP_UART_Referee_Video_Rx_Callback(referee_video_rxbuf, Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_video_rxbuf, referee_video_rxbuf_size);
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_video_rxbuf, referee_video_rxbuf_size);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        is_referee_video_tx_idle = 1;
    }
}
