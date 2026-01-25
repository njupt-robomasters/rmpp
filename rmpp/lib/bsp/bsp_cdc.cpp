#include "bsp_cdc.hpp"
#include <cstdarg>
#include "usb_device.h"
#include "usbd_cdc_if.h"

using namespace BSP;

extern USBD_HandleTypeDef hUsbDeviceFS;

std::vector<CDC::CallbackFunc>* CDC::callbacks;
Dwt CDC::dwt;
UnitFloat<pct> CDC::cpu_usage;

void CDC::Init() {
    MX_USB_DEVICE_Init();
}

bool CDC::IsConnect() {
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

void CDC::Transmit(uint8_t data[], const uint16_t size) {
    CDC_Transmit_FS(data, size);
}

void CDC::printf(const char* format, ...) {
    va_list args;
    char buf[PRINTF_BUF_SIZE];

    va_start(args, format);
    const int len = vsnprintf(buf, PRINTF_BUF_SIZE, format, args);
    va_end(args);

    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
}

void CDC::RegisterCallback(const CallbackFunc& callback) {
    if (!callbacks) {
        callbacks = new std::vector<CallbackFunc>;
    }
    callbacks->push_back(callback);
}

void CDC::InvokeCallback(const uint8_t data[], const uint32_t size) {
    const UnitFloat interval_time = dwt.UpdateDT();
    if (callbacks) {
        for (const auto& callback : *callbacks) {
            callback(data, size);
        }
    }
    const UnitFloat running_time = dwt.GetDT();
    cpu_usage = running_time / interval_time;
}

// 这个函数要放到 USB_DEVICE/App/usbd_cdc_if.c 的 CDC_Receive_FS 函数的结尾调用
extern "C" void BSP_CDC_Callback_ForHAL(const uint8_t data[], const uint32_t size) {
    CDC::InvokeCallback(data, size);
}