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

// 上位机自瞄与导航：通信协议由上游实现，未接入时保持默认值（未识别、导航速度为零）
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(1)});

// 裁判系统经 CAN1 转发模块接入，接收 ID 0x000、发送 ID 0x001
inline Referee referee({.bus = Can(1)});
inline RobotUi ui(referee);

void misc_loop();
