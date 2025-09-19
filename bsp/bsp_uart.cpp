#include "bsp_uart.hpp"
#include "usart.h"
#include <iostream>

using namespace BSP;

std::vector<UART_RC::CallbackFunc>* UART_RC::callbacks;
uint8_t UART_RC::rxbuf[RXBUF_SIZE];

void UART_RC::Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rxbuf, RXBUF_SIZE);
}

void UART_RC::Transmit(const uint8_t data[], const uint16_t size) {
    HAL_UART_Transmit_IT(huart, data, size);
}

void UART_RC::RegisterCallback(const CallbackFunc& callback) {
    if (!callbacks) {
        callbacks = new std::vector<CallbackFunc>;
    }
    callbacks->push_back(callback);
}

void UART_RC::InvokeCallback(const uint16_t size) {
    if (callbacks) {
        for (const auto& callback : *callbacks) {
            callback(rxbuf, size);
        }
    }
}

std::vector<UART_Video::CallbackFunc>* UART_Video::callbacks;
uint8_t UART_Video::rxbuf[RXBUF_SIZE];

void UART_Video::Init() {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rxbuf, RXBUF_SIZE);
}

void UART_Video::Transmit(const uint8_t data[], const uint16_t size) {
    HAL_UART_Transmit_IT(huart, data, size);
}

void UART_Video::RegisterCallback(const CallbackFunc& callback) {
    if (!callbacks) {
        callbacks = new std::vector<CallbackFunc>;
    }
    callbacks->push_back(callback);
}

void UART_Video::InvokeCallback(const uint16_t size) {
    if (callbacks) {
        for (const auto& callback : *callbacks) {
            callback(rxbuf, size);
        }
    }
}

/********************* 以下为HAL库回调函数 ********************/

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    // 遥控器
    if (huart->Instance == USART3) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) { // 串口空闲中断
            HAL_UART_DMAStop(huart); // 停止接收
            UART_RC::InvokeCallback(Size); // 调用回调函数
            UART_RC::Init(); // 继续接收
        } else if (event_type == HAL_UART_RXEVENT_TC) { // 串口DMA完成中断
            UART_RC::Init(); // 继续接收
        }
    }

    // 裁判系统图传发送端串口
    if (huart->Instance == USART6) {
        const uint8_t event_type = HAL_UARTEx_GetRxEventType(huart);
        if (event_type == HAL_UART_RXEVENT_IDLE) { // 串口空闲中断
            HAL_UART_DMAStop(huart); // 停止接收
            UART_Video::InvokeCallback(Size); // 调用回调函数
            UART_Video::Init(); // 继续接收
        } else if (event_type == HAL_UART_RXEVENT_TC) { // 串口DMA完成中断
            UART_Video::Init(); // 继续接收
        }
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART3) { // 遥控器
        __HAL_UNLOCK(huart);
        UART_RC::Init(); // 继续接收
    } else if (huart->Instance == USART6) { // 裁判系统图传发送端串口
        __HAL_UNLOCK(huart);
        UART_Video::Init(); // 继续接收
    }
}
