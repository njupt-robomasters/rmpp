#include "bsp_uart.h"
#include "usart.h"

#include "task_uart.h"

static uint8_t rc_rxbuf_size = 50;
static uint8_t rc_rxbuf[50];

void BSP_UART_RC_Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rxbuf, rc_rxbuf_size);
}

void BSP_UART_RC_Rx_Callback(const uint8_t *data, const uint16_t size) {
    task_uart_rc_callback(data, size);
}

void BSP_UART_Referee_Rx_Callback(const uint8_t *data, uint16_t size) {
}

void BSP_UART_Video_Rx_Callback(const uint8_t *data, uint16_t size) {
}

/********************* 以下为HAL库回调函数 ********************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) {
            // 串口空闲中断
            HAL_UART_DMAStop(&huart3);
            BSP_UART_RC_Rx_Callback(rc_rxbuf, Size);
            BSP_UART_RC_Init();
        } else if (event_type == HAL_UART_RXEVENT_TC) {
            // 串口DMA完成中断
            BSP_UART_RC_Init();
        }
    }
}
