#include "bsp_cdc.h"
#include <stdarg.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

static const int CDC_PRINTF_BUF_SIZE = 128;

// 回调函数
CDC_Callback_t cdc_callback = NULL;

void BSP_CDC_Init() {
    MX_USB_DEVICE_Init();
}

void BSP_CDC_Transmit(uint8_t *data, const uint16_t size) {
    CDC_Transmit_FS(data, size);
}

void BSP_CDC_printf(const char *format, ...) {
    va_list args;
    char buf[CDC_PRINTF_BUF_SIZE];

    va_start(args, format);
    const uint32_t len = vsnprintf(buf, CDC_PRINTF_BUF_SIZE, format, args);
    va_end(args);

    CDC_Transmit_FS((uint8_t *)buf, len);
}

void BSP_CDC_SetCallback(const CDC_Callback_t callback) {
    cdc_callback = callback;
}

static void invokeCDCCallback(const uint8_t *data, const uint32_t size) {
    if (cdc_callback != NULL) {
        cdc_callback(data, size);
    }
}

void BSP_CDC_Callback_ForHAL(const uint8_t *data, const uint32_t size) {
    invokeCDCCallback(data, size);
}
