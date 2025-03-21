#include "bsp_cdc.h"
#include <stdarg.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "task_protocol.h"

static const int CDC_PRINTF_BUF_SIZE = 128;

void BSP_CDC_Init() {
    MX_USB_DEVICE_Init();
}

void BSP_CDC_Transmit(uint8_t *data, const int size) {
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

void BSP_CDC_Rx_Callback(const uint8_t *data, const int size) {
    task_protocol_cdc_callback(data, size);
}
