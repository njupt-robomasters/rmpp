#include "bsp_uart.h"
#include "usart.h"

#define rc_rxbuf_size 50
#define referee_rxbuf_size 127
#define referee_video_rxbuf_size 127

// 接收缓冲区
static uint8_t rc_rxbuf[rc_rxbuf_size];
static uint8_t referee_rxbuf[referee_rxbuf_size];
static uint8_t video_rxbuf[referee_video_rxbuf_size];

// 回调函数
static UART_Callback_t rc_callback = NULL;
static UART_Callback_t referee_callback = NULL;

void BSP_UART_Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rxbuf, referee_video_rxbuf_size);
}

void BSP_UART_Referee_Video_Transmit(const uint8_t *data, const uint16_t size) {
    HAL_UART_Transmit_IT(&huart6, data, size);
}

void BSP_UART_RC_SetCallback(const UART_Callback_t callback) {
    rc_callback = callback;
}

void BSP_UART_Referee_SetCallback(const UART_Callback_t callback) {
    referee_callback = callback;
}

static void invokeRCCallback(const uint8_t *data, const uint16_t size) {
    if (rc_callback != NULL) {
        rc_callback(data, size);
    }
}

static void invokeRefereeCallback(const uint8_t *data, const uint16_t size) {
    if (referee_callback != NULL) {
        referee_callback(data, size);
    }
}

/********************* 以下为HAL库回调函数 ********************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    // 遥控器
    if (huart->Instance == USART3) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart3);
            invokeRCCallback(rc_rxbuf, Size);
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
            invokeRefereeCallback(referee_rxbuf, Size);
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
            invokeRefereeCallback(video_rxbuf, Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rxbuf, referee_video_rxbuf_size);
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rxbuf, referee_video_rxbuf_size);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
    } else if (huart->Instance == USART6) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
    } else if (huart->Instance == USART1) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rxbuf, referee_video_rxbuf_size);
    }
}
