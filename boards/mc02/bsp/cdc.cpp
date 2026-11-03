/*
 * USB 虚拟串口：CherryUSB 设备栈 + DWC2 驱动，OTG_HS 工作在内置全速 PHY（PA11/PA12），USB 时钟 PLL3Q 48 MHz
 *
 * CubeMX 配置：Connectivity → USB_OTG_HS，Internal FS Phy = Device_Only，不启用 USB_DEVICE 中间件，
 * NVIC 不勾选 OTG_HS 中断（由 BSP 开启），Project Manager 中不生成 MX_USB_OTG_HS_PCD_Init 调用
 * 生成的 HAL_PCD_MspInit 开启 USB 电压检测与时钟，由 CherryUSB 的 usb_dc_low_level_init 调用；
 * OTG_HS_IRQHandler 由 CherryUSB usb_glue_st.c 定义
 *
 * 两个 CDC 以 IAD 复合设备枚举：Cdc(1) 为接口 0–1（EP1 批量，EP3 通知），Cdc(2) 为接口 2–3（EP2 批量，EP4 通知）
 * DWC2 工作在非 DMA 模式，CPU 读写 FIFO，收发缓冲无需 D-Cache 维护，可留在 DTCM
 */

#include "bsp/cdc.hpp"
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_acm.h"
#include "usbd_core.h"
#include <cstring>
#include <vector>

// 描述符常量：VID/PID 为 pid.codes 测试 ID，仅限内部使用，避免被 ST VCP 驱动按 0483:5740 整机绑定；字符串沿用 CubeMX 虚拟串口默认值
static constexpr uint16_t VID = 0x1209;
static constexpr uint16_t PID = 0x0001;
static constexpr uint16_t BCD_DEVICE = 0x0200;
static constexpr const char* MANUFACTURER = "STMicroelectronics";
static constexpr const char* PRODUCT = "STM32 Virtual ComPort";
static constexpr int MAX_POWER_MA = 100;

static constexpr uint8_t BUS = 0;
static constexpr uint16_t MAX_PACKET_SIZE = 64; // 全速批量端点
static constexpr std::size_t BUFFER_CAPACITY = 2048;

struct CdcContext {
    int id;
    uint8_t out_ep, in_ep;          // 批量端点；中断通知端点只出现在描述符中
    usbd_interface interfaces[2]{}; // 控制接口、数据接口
    usbd_endpoint endpoints[2]{};   // 批量 OUT、批量 IN
    alignas(4) uint8_t rx_buf[BUFFER_CAPACITY]{};
    alignas(4) uint8_t tx_buf[BUFFER_CAPACITY]{};
    volatile bool is_tx_busy = false;
    std::vector<Cdc::RxCallback> callbacks;
    Monitor monitor;
};

static constinit CdcContext contexts[] = {
    {.id = 1, .out_ep = 0x01, .in_ep = 0x81}, // 接口 0–1
    {.id = 2, .out_ep = 0x02, .in_ep = 0x82}, // 接口 2–3
};

static constexpr uint8_t DEVICE_DESCRIPTOR[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, VID, PID, BCD_DEVICE, 0x01),
};

static constexpr uint8_t CONFIG_DESCRIPTOR[] = {
    USB_CONFIG_DESCRIPTOR_INIT(9 + 2 * CDC_ACM_DESCRIPTOR_LEN, 0x04, 0x01, USB_CONFIG_BUS_POWERED, MAX_POWER_MA),
    CDC_ACM_DESCRIPTOR_INIT(0x00, 0x83, 0x01, 0x81, MAX_PACKET_SIZE, 0x00),
    CDC_ACM_DESCRIPTOR_INIT(0x02, 0x84, 0x02, 0x82, MAX_PACKET_SIZE, 0x00),
};

// 序列号：同 CubeMX Get_SerialNum，UID0 + UID2 取 8 位十六进制，UID1 取高 4 位
static char serial[13];

static void make_serial() {
    const auto* uid = reinterpret_cast<const uint32_t*>(UID_BASE);
    const uint32_t high = uid[0] + uid[2];
    const uint32_t low = uid[1];
    static constexpr char HEX[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++)
        serial[i] = HEX[(high >> (28 - 4 * i)) & 0xF];
    for (int i = 0; i < 4; i++)
        serial[8 + i] = HEX[(low >> (28 - 4 * i)) & 0xF];
}

static const uint8_t* device_descriptor(uint8_t) {
    return DEVICE_DESCRIPTOR;
}

static const uint8_t* config_descriptor(uint8_t) {
    return CONFIG_DESCRIPTOR;
}

static const char* string_descriptor(uint8_t, const uint8_t index) {
    static constexpr char LANGID[] = {0x09, 0x04}; // 英语（美国）
    switch (index) {
    case USB_STRING_LANGID_INDEX:
        return LANGID;
    case USB_STRING_MFC_INDEX:
        return MANUFACTURER;
    case USB_STRING_PRODUCT_INDEX:
        return PRODUCT;
    case USB_STRING_SERIAL_INDEX:
        return serial;
    default:
        return nullptr;
    }
}

static constexpr usb_descriptor DESCRIPTOR = {
    .device_descriptor_callback = device_descriptor,
    .config_descriptor_callback = config_descriptor,
    .string_descriptor_callback = string_descriptor,
};

static CdcContext* find(const int id) {
    for (auto& context : contexts)
        if (context.id == id)
            return &context;
    return nullptr;
}

static CdcContext* find_by_ep(const uint8_t ep) {
    for (auto& context : contexts)
        if (context.out_ep == ep || context.in_ep == ep)
            return &context;
    return nullptr;
}

// 批量 OUT 完成：通知订阅者后继续接收
static void on_bulk_out(const uint8_t busid, const uint8_t ep, const uint32_t size) {
    auto* context = find_by_ep(ep);

    context->monitor.begin();
    for (const auto& callback : context->callbacks)
        callback(context->rx_buf, size);
    context->monitor.end();

    usbd_ep_start_read(busid, ep, context->rx_buf, BUFFER_CAPACITY);
}

// 批量 IN 完成：长度为包长整数倍时补发零长包结束传输
static void on_bulk_in(const uint8_t busid, const uint8_t ep, const uint32_t size) {
    auto* context = find_by_ep(ep);
    if (size > 0 && size % MAX_PACKET_SIZE == 0)
        usbd_ep_start_write(busid, ep, nullptr, 0);
    else
        context->is_tx_busy = false;
}

static void on_event(const uint8_t busid, const uint8_t event) {
    if (event != USBD_EVENT_CONFIGURED)
        return;

    for (auto& context : contexts) {
        context.is_tx_busy = false;
        usbd_ep_start_read(busid, context.out_ep, context.rx_buf, BUFFER_CAPACITY);
    }
}

void cdc_init() {
    make_serial();
    usbd_desc_register(BUS, &DESCRIPTOR);

    for (auto& context : contexts) {
        context.endpoints[0] = {.ep_addr = context.out_ep, .ep_cb = on_bulk_out};
        context.endpoints[1] = {.ep_addr = context.in_ep, .ep_cb = on_bulk_in};
        usbd_add_interface(BUS, usbd_cdc_acm_init_intf(BUS, &context.interfaces[0]));
        usbd_add_interface(BUS, usbd_cdc_acm_init_intf(BUS, &context.interfaces[1]));
        usbd_add_endpoint(BUS, &context.endpoints[0]);
        usbd_add_endpoint(BUS, &context.endpoints[1]);
    }

    usbd_initialize(BUS, USB_OTG_HS_PERIPH_BASE, on_event);
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void Cdc::send(const uint8_t* data, const std::size_t size) const {
    auto* context = find(id_);
    if (!context)
        return;

    // 检查并占用发送缓冲，防止主循环与中断回调同时发送
    {
        const IrqLock lock;
        if (!usb_device_is_configured(BUS) || context->is_tx_busy)
            return;
        context->is_tx_busy = true;
    }

    std::memcpy(context->tx_buf, data, size);
    if (usbd_ep_start_write(BUS, context->in_ep, context->tx_buf, size) != 0)
        context->is_tx_busy = false;
}

void Cdc::add_rx_callback(RxCallback callback) const {
    auto* context = find(id_);
    if (!context)
        return;

    const IrqLock lock;
    context->callbacks.push_back(std::move(callback));
}
