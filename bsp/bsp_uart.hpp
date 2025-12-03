#pragma once

#include <functional>
#include <vector>

#include <usart.h>

namespace BSP {
    template<typename T>
    class UART {
    protected:
        using CallbackFunc = std::function<void(const uint8_t data[], uint16_t size)>;

    public:
        static void Init() {
            HAL_UARTEx_ReceiveToIdle_DMA(T::huart, T::rxbuf, T::RXBUF_SIZE);
        }

        static void Transmit(const uint8_t data[], uint16_t size) {
            HAL_UART_Transmit_IT(T::huart, data, size);
        }

        static void RegisterCallback(const CallbackFunc &callback) {
            if (!T::callbacks) {
                T::callbacks = new std::vector<CallbackFunc>;
            }
            T::callbacks->push_back(callback);
        }

        static void InvokeCallback(uint16_t size) {
            if (T::callbacks) {
                for (const auto &callback: *T::callbacks) {
                    callback(T::rxbuf, size);
                }
            }
        }
    };


    class UART3 : public UART<UART3> {
        friend class UART;

    private:
        static constexpr UART_HandleTypeDef *huart = &huart3;
        static constexpr uint8_t RXBUF_SIZE = 50;

        static std::vector<CallbackFunc> *callbacks; // 保存注册的回调函数
        static uint8_t rxbuf[RXBUF_SIZE];
    };

    class UART6 : public UART<UART6> {
        friend class UART;

    private:
        static constexpr UART_HandleTypeDef *huart = &huart6;
        static constexpr uint8_t RXBUF_SIZE = 128;

        static std::vector<CallbackFunc> *callbacks; // 保存注册的回调函数
        static uint8_t rxbuf[RXBUF_SIZE];
    };
}
