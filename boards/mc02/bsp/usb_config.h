#pragma once

// CherryUSB 配置，完整选项见 lib/third_party/CherryUSB/cherryusb_config_template.h

// 不输出协议栈日志
#define CONFIG_USB_PRINTF(...)
#define CONFIG_USB_DBG_LEVEL USB_DBG_ERROR

// DWC2 工作在非 DMA 模式，CPU 读写 FIFO，缓冲区不受 D-Cache 影响，4 字节对齐即可
#define CONFIG_USB_ALIGN_SIZE 4
#define USB_NOCACHE_RAM_SECTION
#define usb_phyaddr2ramaddr(addr) (addr)
#define usb_ramaddr2phyaddr(addr) (addr)

// 使用标准库 memcpy；否则 usb_memcpy.h 把 memcpy 宏替换为 usb_memcpy，破坏 C++ 的 std::memcpy
#define CONFIG_USB_MEMCPY_DISABLE

// 设备栈：单总线，EP0 缓冲区容纳配置描述符和字符串
#define CONFIG_USBDEV_MAX_BUS 1
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512

// usb_glue_st.c 引用主机栈头文件，只用于确定结构体尺寸
#define CONFIG_USBHOST_MAX_RHPORTS 1
#define CONFIG_USBHOST_MAX_EXTHUBS 1
#define CONFIG_USBHOST_MAX_EHPORTS 4
#define CONFIG_USBHOST_MAX_INTERFACES 8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 2
#define CONFIG_USBHOST_MAX_ENDPOINTS 4
#define CONFIG_USBHOST_DEV_NAMELEN 16
