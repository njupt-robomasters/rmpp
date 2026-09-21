#include "board/cdc.hpp"
#include "board/timer.hpp"
#include "irq_lock.hpp"
#include "peripherals.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace rmpp {
struct Cdc::State {
    std::vector<RxCallback> callbacks;
    std::array<std::uint8_t, 2048> tx{};
    Timer timer{0};
    float cpu_usage = 0;
};

Cdc::State* Cdc::find(int id) {
    if (id != 1)
        return nullptr;
    static constinit State state;
    return &state;
}

bool Cdc::send(Bytes bytes) const {
    if (!state_ || !bytes.data || !bytes.size || bytes.size > state_->tx.size())
        return false;
    IrqLock lock;
    auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || !cdc || cdc->TxState)
        return false;
    std::memcpy(state_->tx.data(), bytes.data, bytes.size);
    return CDC_Transmit_FS(state_->tx.data(), bytes.size) == USBD_OK;
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
    IrqLock lock;
    state_->callbacks.push_back(std::move(callback));
    return true;
}

float Cdc::cpu_usage() const { return state_ ? state_->cpu_usage : 0; }

void cdc_receive(Bytes bytes) {
    auto* state = Cdc::find(1);
    const float interval = state->timer.update_dt();
    for (const auto& callback : state->callbacks)
        callback(bytes);
    state->cpu_usage = interval > 0 ? state->timer.get_dt() / interval : 0;
}
} // namespace rmpp

extern "C" void bsp_cdc_receive(const uint8_t* data, uint32_t size) {
    rmpp::cdc_receive({data, size});
}
