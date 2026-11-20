#pragma once

#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/rgb.hpp"
#include "bsp/uart.hpp"

#include "bsp/kv_store.hpp"
#include "melody/melody.hpp"
#include "minipc/aim.hpp"
#include "minipc/nav.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "referee/referee.hpp"
#include "referee/ui.hpp"

// 板载 WS2812 与蜂鸣器
inline Rgb rgb;
inline Melody melody({});

// 持久化参数（W25Q64），当前只保存大 yaw 零位 yaw_offset
inline KvStore kv_store("jelly_sentry");

// 遥控器：VT13 图传接 USART1（921600），FSi6X 接收机 SBUS 接 UART5
inline VT13 vt13({.uart = Uart(1)});
inline FSi6X fsi6x({.uart = Uart(5)});

// 上位机独立通道：CDC1 自瞄（21 字节上行 / 18 字节下行），CDC2 导航（17 字节下行）
inline Aim aim(Cdc(1), {.timeout = 200 * ms});
inline Nav nav(Cdc(2), {});

// 裁判系统经 CAN1 转发模块接入，接收 ID 0x000、发送 ID 0x001
inline Referee referee({.bus = Can(1)});
inline RobotUi ui(referee);

void misc_loop();
