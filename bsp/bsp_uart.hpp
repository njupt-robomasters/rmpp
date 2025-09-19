#pragma once

#include <functional>
#include <vector>

#include <usart.h>

namespace BSP {
    class UART_RC {
        using CallbackFunc = std::function<void(const uint8_t data[], uint16_t size)>;

        static std::vector<CallbackFunc>* callbacks; // 保存注册的回调函数

        // 串口接收缓冲区相关
        static constexpr UART_HandleTypeDef* huart = &huart3;
        static constexpr uint8_t RXBUF_SIZE = 50;
        static uint8_t rxbuf[RXBUF_SIZE];

    public:
        static void Init();

        static void Transmit(const uint8_t data[], uint16_t size);

        static void RegisterCallback(const CallbackFunc& callback);

        static void InvokeCallback(uint16_t size);
    };;

    class UART_Video {
        using CallbackFunc = std::function<void(const uint8_t data[], uint16_t size)>;

        static std::vector<CallbackFunc>* callbacks; // 保存注册的回调函数

        // 串口接收缓冲区相关
        static constexpr UART_HandleTypeDef* huart = &huart6;
        static constexpr uint8_t RXBUF_SIZE = 128;
        static uint8_t rxbuf[RXBUF_SIZE];

    public:
        static void Init();

        static void Transmit(const uint8_t data[], uint16_t size);

        static void RegisterCallback(const CallbackFunc& callback);

        static void InvokeCallback(uint16_t size);
    };
}
