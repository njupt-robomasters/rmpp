#pragma once
#include "usb_device.h"
inline const std::uint8_t* usb_tx = nullptr;
inline std::uint16_t usb_tx_size = 0;
inline std::uint8_t CDC_Transmit_FS(std::uint8_t* data, std::uint16_t size) {
    usb_tx = data;
    usb_tx_size = size;
    static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData)->TxState = 1;
    return USBD_OK;
}
extern "C" void bsp_cdc_receive(const std::uint8_t*, std::uint32_t);
