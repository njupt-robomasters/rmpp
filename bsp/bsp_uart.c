#include "bsp_uart.h"
#include "usart.h"
#include "task_uart.h"

static uint8_t rc_rxbuf_size = 50;
static uint8_t rc_rxbuf[50];
static uint8_t referee_rxbuf_size = 127;
static uint8_t referee_rxbuf[127];
static uint8_t video_rx_buf[127];
static uint8_t video_rxbuf_size = 127;

void BSP_UART_Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, referee_rxbuf, referee_rxbuf_size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rx_buf, video_rxbuf_size);
}

void BSP_UART_RC_Rx_Callback(const uint8_t *data, const uint16_t size) {
    task_uart_rc_callback(data, size);
}

void BSP_UART_Referee_Rx_Callback(const uint8_t *data, uint16_t size) {
    task_uart_referee_callback(data, size);
}

void BSP_UART_Video_Rx_Callback(const uint8_t *data, uint16_t size) {
    task_uart_referee_callback(data, size);
}

/********************* 以下为HAL库回调函数 ********************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
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

    if (huart->Instance == USART1) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart1);
            BSP_UART_Video_Rx_Callback(video_rx_buf, Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rx_buf, video_rxbuf_size);
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, video_rx_buf, video_rxbuf_size);
        }
    }
}
