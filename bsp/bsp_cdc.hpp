#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace BSP {
    class CDC {
        static constexpr int PRINTF_BUF_SIZE = 128;

        using CallbackFunc = std::function<void(const uint8_t data[], uint32_t size)>;

        static std::vector<CallbackFunc>* callbacks; // 保存注册的回调函数

    public:
        static void Init();

        static bool IsConnect();

        static void Transmit(uint8_t data[], uint16_t size);

        static void printf(const char* format, ...);

        static void RegisterCallback(const CallbackFunc& callback);

        static void InvokeCallback(const uint8_t data[], uint32_t size);
    };
}
