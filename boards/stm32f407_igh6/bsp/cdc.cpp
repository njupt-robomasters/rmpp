/*
 * USB 虚拟串口：Connectivity → USB_OTG_FS
 *
 * | 页面                    | 参数                       | 设置                                      |
 * | ----------------------- | -------------------------- | ----------------------------------------- |
 * | USB_OTG_FS              | Mode                       | Device_Only                               |
 * | Middleware → USB_DEVICE | Class For FS IP            | Communication Device Class (Virtual Port Com) |
 * | NVIC Settings           | USB On The Go FS interrupt | 勾选，Preemption Priority=0               |
 *
 * | 通道      | DM   | DP   |
 * | --------- | ---- | ---- |
 * | USB_OTG_FS | PA11 | PA12 |
 */

#include "bsp/cdc.hpp"
#include "bsp/monitor.hpp"
#include "usbd_cdc_if.h"
#include <cstring>

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

struct Cdc::Context {
    int id;
    std::vector<RxCallback> callbacks;
    Monitor monitor;
};

Cdc::Cdc(const int id) {
    static constinit Context contexts[] = {
        {.id = 1},
    };
    if (id == 1)
        context_ = &contexts[0];
}

bool Cdc::send(const uint8_t* data, const size_t size) const {
    if (!context_)
        return false;

    // USB 未枚举完成时 pClassData 为空，静默丢弃
    const auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || !cdc || !cdc->TxBuffer || cdc->TxState)
        return false;

    // CDC_Init_FS 将 TxBuffer 绑定到生成的 UserTxBufferFS
    std::memcpy(cdc->TxBuffer, data, size);
    return CDC_Transmit_FS(cdc->TxBuffer, size) == USBD_OK;
}

void Cdc::add_rx_callback(RxCallback callback) const {
    if (!context_)
        return;

    const auto primask = __get_PRIMASK();
    __disable_irq();
    context_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
}

extern "C" void bsp_cdc_receive(const uint8_t* data, uint32_t size) {
    Cdc::Context* context = Cdc(1).context_;

    context->monitor.begin();
    for (const auto& callback : context->callbacks)
        callback(data, size);
    context->monitor.end();
}
