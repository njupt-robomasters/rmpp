#include "bsp/cdc.hpp"
#include "peripherals.hpp"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

extern USBD_HandleTypeDef hUsbDeviceFS;

struct Cdc::State {
    int id;
    std::vector<RxCallback> callbacks;
    std::array<std::uint8_t, 2048> tx{};

    struct Metrics {
        float last_callback_us = 0;
        float cpu_usage = 0;
    };

    volatile Metrics metrics{};
    CallbackTiming timing;
};

Cdc::Cdc(int id) : id_(id) {
    static constinit State states[] = {
        {.id = 1},
    };
    for (auto& state : states) {
        if (state.id == id_) {
            state_ = &state;
            break;
        }
    }
}

bool Cdc::send(Bytes bytes) const {
    if (!state_ || !bytes.data || !bytes.size || bytes.size > state_->tx.size())
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    bool success = false;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && !cdc->TxState) {
        std::memcpy(state_->tx.data(), bytes.data, bytes.size);
        success = CDC_Transmit_FS(state_->tx.data(), bytes.size) == USBD_OK;
    }
    __set_PRIMASK(primask);
    return success;
}

bool Cdc::printf(const char* format, ...) const {
    if (!state_ || !format)
        return false;
    char buffer[128];
    va_list args;
    va_start(args, format);
    const int size = std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (size <= 0)
        return false;
    const auto length = static_cast<std::size_t>(size) < sizeof(buffer) ? size : sizeof(buffer) - 1;
    return send({reinterpret_cast<const std::uint8_t*>(buffer), length});
}

bool Cdc::add_rx_callback(RxCallback callback) const {
    if (!state_ || !callback)
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    state_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
    return true;
}

void cdc_receive(Bytes bytes) {
    auto* state = Cdc(1).state_;
    const auto begin = DWT->CYCCNT;
    for (const auto& callback : state->callbacks)
        callback(bytes);
    const float duration_us = static_cast<std::uint32_t>(DWT->CYCCNT - begin) * (1000000.0f / SystemCoreClock);
    state->metrics.last_callback_us = duration_us;
    state->timing.record(duration_us, HAL_GetTick(), state->metrics.cpu_usage);
}

extern "C" void bsp_cdc_receive(const uint8_t* data, uint32_t size) {
    cdc_receive({data, size});
}
