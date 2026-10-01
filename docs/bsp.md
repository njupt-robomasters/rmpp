# BSP 开发约定

仅在修改 `lib/bsp/**` 或 `boards/**/bsp/**` 时读取本文档

## 构造与启动

- Can、Uart、Cdc、Pwm、Spi 构造时只绑定 Context
- 硬件启动由板卡初始化入口执行
- 外设操作由对应外设类实现
- 同通道共享 Context
- Timer 独立计时
- Storage 共享唯一 Context，首次构造时初始化数据库

## 初始化入口

- 仅供板卡入口调用的初始化函数在板卡 `bsp.cpp` 中直接 `extern` 声明
- 不为这些函数另设公共头文件
- C 与 C++ 共用入口使用 `extern "C"`
- 文件内部处理函数使用 `static`
- 文件内部函数不提供独立公开声明

## 句柄与接口

- Can、Spi 必须显式指定硬件
- 不提供默认空句柄
- 型号默认参数不包含硬件句柄
- Context 不存在时安全失败
- 外设不提供 `operator bool()`
- 发送接口返回请求是否被底层接受
- 参数范围由调用方保证，不重复检查
- 必要的运行状态判断留在实现中
- CDC 未枚举时丢弃发送
- 数值读取和实际设置值可作为返回值
- 调试统计通过 `monitor` 查看

## Context 与 Monitor

- 内部运行数据统一为 `Context/context_`
- 共用调试统计统一为 `Monitor/monitor`
- Monitor 放在 `lib/bsp/monitor.hpp`
- 同类指标和计时合并复用
- 不另分 timing 与 metrics

## 缓冲

- 固定长度协议数据和收发缓冲优先使用原生数组
- 缓冲命名为 `rx_buf`、`tx_buf`
- 已有库缓冲直接复用
- CDC 使用 STM32 CDC 库缓冲，不在 BSP 重复创建

## DMA 与 CubeMX

- 硬件 UART 收发统一使用 DMA
- 所需 DMA 必须在 CubeMX 配齐
- 不回退到逐字节中断收发
- 时钟、GPIO、DMA 句柄关联及 NVIC/IRQHandler 由 CubeMX 生成
- BSP 不手写替代初始化

## 配置注释

- 有 CubeMX 配置要求的外设文件头使用 `/* ... */` 多行注释
- 用表格列出页面、界面参数名、设置值和勾选项
- 通道与引脚映射另列表
- 无专用外设配置的实现不补空泛配置表
