#include "bsp_uart.hpp"
#include "usart.h"

using namespace BSP;

std::vector<UART3::CallbackFunc>* UART3::callbacks;
uint8_t UART3::rxbuf[RXBUF_SIZE];

std::vector<UART6::CallbackFunc>* UART6::callbacks;
uint8_t UART6::rxbuf[RXBUF_SIZE];


/********************* 以下为HAL库回调函数 ********************/

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    // 遥控器串口
    if (huart->Instance == USART3) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) { // 串口空闲中断
            HAL_UART_DMAStop(huart); // 停止接收
            UART3::InvokeCallback(Size); // 调用回调函数
            UART3::Init(); // 继续接收
        } else if (event_type == HAL_UART_RXEVENT_TC) { // 串口DMA完成中断
            UART3::Init(); // 继续接收
        }
    }

    // 裁判系统图传串口
    if (huart->Instance == USART6) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) { // 串口空闲中断
            HAL_UART_DMAStop(huart); // 停止接收
            UART6::InvokeCallback(Size); // 调用回调函数
            UART6::Init(); // 继续接收
        } else if (event_type == HAL_UART_RXEVENT_TC) { // 串口DMA完成中断
            UART6::Init(); // 继续接收
        }
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART3) { // 遥控器串口
        __HAL_UNLOCK(huart);
        UART3::Init(); // 继续接收
    } else if (huart->Instance == USART6) { // 裁判系统图传串口
        __HAL_UNLOCK(huart);
        UART6::Init(); // 继续接收
    }
}
