#pragma once

#include <functional>
#include <vector>
#include "bsp/bsp_dwt.h"
#include "unit/include_me.hpp"

namespace BSP {
    class CAN {
    private:
        using CallbackFunc = std::function<void(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc)>;

        static std::vector<CallbackFunc>* callbacks; // 保存注册的回调函数

        static Dwt dwt;

    public:
        static UnitFloat<pct> cpu_usage;
        static uint32_t bus_off_cnt;

        static void Init();

        static void TransmitStd(uint8_t port, uint32_t id, uint8_t data[8], uint8_t dlc = 8);

        static void TransmitExt(uint8_t port, uint32_t id, uint8_t data[8], uint8_t dlc = 8);

        static void RegisterCallback(const CallbackFunc& callback);

        static void InvokeCallback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
    };
}