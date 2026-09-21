#pragma once
#include <cstdint>
inline constexpr unsigned USBD_STATE_CONFIGURED = 1, USBD_OK = 0;
struct USBD_HandleTypeDef {
    unsigned dev_state = 0;
    void* pClassData = nullptr;
};
struct USBD_CDC_HandleTypeDef {
    unsigned TxState = 0;
};
inline USBD_HandleTypeDef hUsbDeviceFS;
