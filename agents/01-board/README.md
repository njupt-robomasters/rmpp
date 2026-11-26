# 01 板卡与底层

状态：两板 BSP 已并入 master；EL05/CAN2 等硬件问题未闭环，见 `../TODO.md`

C 板（STM32F407）与 mc02（STM32H723）的 BSP、CubeMX 配置、构建、IMU 和电机通信。

## 文档

| 文件 | 内容 |
| --- | --- |
| `bsp.md` | BSP 接口、初始化、回调、IrqLock、DMA 与 CubeMX 配置 |
| `build.md` | CMake 结构、工具链、生成器、构建检查 |
| `imu.md` | BMI088 配置、坐标转换、校准与解算 |
| `imu-principles.md` | EKF 原理，按公式逐段对应代码 |
| `motor.md` | 电机库构造与配置、角度零点、DJI/DM/EL05/VESC 协议 |
| `../02-conventions/architecture.md` | 目录、启动与句柄、板卡资源、格式化 |

## 任务记录

- `mc02_bsp/`：mc02 适配、EL05 私有扩展帧、CAN2 降载、7025/PCAN 干扰、Horco 探针与两路电机掉线
- `attitude_cpu/`：Eigen 在 -Og 下不内联导致 CPU 超时，结论为 EKF 按 O2 编译
- `can_tx_latest/`：同 ID 只留最新方案已撤销，保留 CAN 撤销与 FIFO 的总线实测结论

## 当前要点

- 板卡资源：C 板 CAN1/2、UART1/3/6、CDC1；mc02 CAN1/2/3（FDCAN 经典帧）、UART1/5/7/10、CDC1/2
- 第三方库：`lib/third_party/Eigen`；`third_party/CherryUSB`、FlashDB 2.2.0
- KvStore 构造不访问 Flash，首次读写初始化；C 板内部 Flash 768 KiB 偏移、256 KiB 长度，mc02 为 W25Q64 从 0 起 1 MiB
- 电机电流与力矩按 `max_torque/max_current` 换算，减速比、方向和零点在输出侧修正；角度统一命名 `angle`、`angle_multi`，单位 rad

## 待办

EL05/CAN2 干扰、DM4310 掉反馈、mc02 OUT_5V 归属、KvStore 升级后的 yaw_offset 重标、蜂鸣器上板，详见 `../TODO.md`
