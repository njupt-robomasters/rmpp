# 规划与待办

跨对话保存规划阶段已确认的决定、待确认问题和后续任务。规划时同步更新；任务完成后把长期结论提炼进 `docs/`，再从本文件删除

## mc02 BSP 适配与 C 板优化

状态：代码与文档完成，编译通过，待用户审计和上板。结论见 `docs/bsp.md`，过程与待用户操作见 `tasks/01-mc02_bsp/README.md`

- mc02 `main.c` USER CODE 2 的 `bsp_init()`/`app_main()` 调用尚未添加（用户操作）
- OUT_5V 供电的是哪一路 CAN 接口：待看原理图图片或实测后写入 `board/power_hpp` 注释

## 第二轮：KvStore、FlashDB 重建、目录整理

状态：在 worktree `../rmpp-kvstore` 的 kvstore 分支实施；步骤 1–2 已完成并编译通过，步骤 3 等待用户在 CubeMX 配置 OCTOSPI1

### 设计要求（已写入 AGENTS.md）

- `lib/`（含 `lib/third_party/`）不依赖 STM32 HAL、CMSIS 或板卡头文件
- 当前唯一违反项：`lib/third_party/CherryUSB/port/dwc2`（`usb_glue_st.c` 引用 `stm32*_hal.h` 与 CMSIS）
- 处理：第三方库按是否含硬件分两处
  - `lib/third_party/`：纯软件（Eigen）
  - 根目录 `third_party/`：含硬件移植层、只由 `boards/*/bsp` 编译的库（CherryUSB、FlashDB）
  - 用户已确认根目录 `third_party/`

### FlashDB（从原版重建）

- 现状：C 板拷贝版本号 2.1.99（开发快照）；与 2.2.0 对比，`fdb.c`、`flashdb.h` 及 FAL 全部相同，`fdb_kvdb.c`、`fdb_tsdb.c`、`fdb_utils.c`、`fdb_def.h`、`fdb_low_lvl.h` 不同；`fal_flash_stm32f4.c` 与上游示例完全一致
- 用户已同意升级到 2.2.0。方案：用上游 2.2.0 原样，只拷 `inc/`、`src/fdb.c fdb_kvdb.c fdb_utils.c`、`port/fal/inc`、`port/fal/src/fal.c fal_flash.c fal_partition.c`、LICENSE；不拷 tsdb、fdb_file、fal_rtt、demos
- 库本身不依赖 STM32（rtthread、unistd 仅在对应宏下包含），但 `flashdb.h` 包含板卡 `fdb_cfg.h`、FAL 包含 `fal_cfg.h`，只能由 BSP 编译，不进 `rmpp_lib`
- 不改 FlashDB 源码：上游用 `#include <fdb_cfg.h>`、`#include <fal_cfg.h>` 尖括号包含，设计上就是由使用者通过头文件搜索路径提供配置；各板 CMake 的 `flashdb` 目标把 `third_party/FlashDB/inc`、`port/fal/inc` 和本板配置目录加入 include 路径，与 CherryUSB 的 `usb_config.h` 同法
- Flash 设备符号（`fal_flash_dev`）由板卡驱动定义，在本板 `fal_cfg.h` 的设备表里引用；`FDB_PRINT` 等可覆盖宏也在板卡配置里定义
- 只有板卡 `kv_store.cpp` 包含 `flashdb.h`；`lib/bsp/kv_store.hpp` 不包含 FlashDB 头文件
- 各板 `bsp/` 自带：`fdb_cfg.h`、`fal_cfg.h`、Flash 驱动、`kv_store.cpp`；各板 CMake 创建 `flashdb` OBJECT 目标
  - C 板：驱动用上游 `port/fal/samples/porting/fal_flash_stm32f4.c`（示例，放板卡），`FDB_WRITE_GRAN 8`，分区不变（768 KiB 偏移、256 KiB）
  - mc02：OCTOSPI1 + W25Q64 手写 FAL 驱动（JEDEC 0xEF4017，0x03 读，0x06+0x02 页编程按 256 字节拆分，0x20 扇区擦除，轮询 WIP），`FDB_WRITE_GRAN 1`，分区 0 起 1 MiB、扇区 4 KiB；CPU 轮询，不用 DMA
  - mc02 CubeMX（用户操作）：OCTOSPI1 单线模式，引脚参考达妙例程 PB2/PE11/PD11/PB0/PA3/PA1，时钟 ≤ 50 MHz 起步，不开 DMA 与中断
- 升级版本会改变存储格式风险小但未验证；C 板已有的 yaw_offset 可能需重新标定，上板时核对

### KvStore（Storage 改名）

- `lib/bsp/storage.hpp` → `lib/bsp/kv_store.hpp`，类 `KvStore`，`read`/`write` 保留；构造参数仍为应用标记，注释强调更换时格式化
- 迁移：两板实现、gimbal `load/save_yaw_offset(KvStore&)`、infantry/sentry `misc.hpp`、文档

### sentry 大 yaw：MF9025 → DM4310

- `app/sentry/gimbal.hpp` 的 `yaw1` 改为 `DM4310`，输出改为力矩；变更 `send()` 调用保持
- 用户确认可以随写，后续自行调整：暂用 Can(1)，tx 0x12 / rx 0x13（避开 pitch 的 0x11/0x10）；`yaw1_pid`、安装方向保持原值，加注释标明待实车整定
- `GimbalDualYaw` 的 `yaw_output` 同时作用于 yaw1（DM4310）和 yaw2（GM6020），不在本轮修改，写入 sentry 注释提醒

### 目录整理

- `lib/motor` 展平：`dji/`、`dm/` 下文件移至 `lib/motor/`，git mv，迁移 include

### 顺序

1. 目录整理（motor、melody）与 sentry DM4310
2. 第三方库搬到根 `third_party/`；FlashDB 2.2.0 原版重建 + C 板 KvStore
3. mc02 OCTOSPI 配置完成后实现 W25Q64 驱动与 KvStore

## 蜂鸣器复音播放器进 lib

状态：代码与文档完成，三个目标编译通过，待用户审计与上板。结论见 `docs/music.md`

- 待上板：mc02 `examples-melody`；C 板复音算力未实测（infantry/sentry 仍只用普通模式）

## 四舵轮底盘

状态：代码完成并合入 master，编译通过，待用户审计与上车。结论与上车核对项见 `tasks/03-quad_steer/README.md`

## 轮腿底盘迁移

状态：代码完成，`leg` 编译通过，待用户审计与上车。worktree `../rmpp-leg-chassis`，分支 `leg_chassis`；结论与上车核对项见 `tasks/04-leg_chassis/README.md`

- 待做：虚拟 IMU（云台 IMU + 关节角）；云台跟随

## 超级电容 SuperCap

状态：代码完成，`rmpp_lib` 编译通过，待用户审计与上车。协议来源 njupt-robomasters/G474SuperCap master@587146a（私有仓库）

### 协议

- 主控→超电 0x061：`data[0].bit0` 使能 DCDC，`data[1..2]` 电池侧目标功率 int16 大端 0.01 W/LSB
- 超电→主控 0x051：仅在收到 0x061 时应答；V_BAT、I_OUT、V_CAP、I_CAP，int16 大端 mV/mA
- 固件 100 ms 未收到命令关 DCDC；DCDC 使能后稳定电池侧功率，负载超过目标即自动放电，无“只充”开关；电容 8 V 止放、8–9 V 斜坡
- 不回传 I_BAT，可测为底盘口负载功率 V_BAT·I_OUT（含舵电机）

### 已确认

- 类 `SuperCap`，`lib/power/super_cap.{hpp,cpp}`；模式 OFF / CHARGE / BOOST；应答 `voltage`、`current`、`power` 的 chassis/cap
- 裁判输入 `set_referee_limit()`、`set_referee_energy()`；电池侧目标 `power_battery_ref` = 裁判上限·g(裁判缓冲能量)，所有模式下发
- 底盘功率目标 `power_chassis_ref` = 供给·g(所用缓冲能量)；g 为 30 J 零、60 J 1.1 倍线性
- OFF、离线、不限功率：供给为裁判上限，用 `energy_referee`，`energy_local` 跟随
- CHARGE / BOOST：供给为 battery_ref（+ boost_max），`energy_local += (供给 − power_chassis)·dt`，限幅 [0, 60] J；电容额外放电与电容放空后的电池超用都扣除
- 已删 MONITOR、`set_estimate()`、估计偏差修正；底盘保留 `power.estimate` 供 SWD 对照
- dummy：SWC 上 OFF、中 CHARGE、下 BOOST，遥控断联 OFF；referee_loop 模拟裁判，超电 CAN1

### 待上车核对

- 0x051 应答电压电流符号与量程；底盘 `power.estimate` 与 `super_cap.power_chassis` 对比
- CHARGE 稳态 `power_cap` 不为负；若因 DCDC 损耗偏负再考虑充电余量；BOOST 在 10/12 V 的补偿衰减
- infantry/sentry 未装超电（不调用 send，按 OFF）

## 底盘深度重构、PID 回算、UI 下放

状态：代码完成，受影响目标全部编译通过，待用户审计与上车。结论与上车核对项见 `tasks/03-quad_steer/README.md`

## 全局代码优化

状态：代码与文档完成，15 个预设全部编译通过，待用户审计与上车。基线 `8f01151`，过程与上车核对项见 `tasks/10-global_refactor/README.md`

## 后续任务

- VQF 姿态算法：原样引入官方 C++ 实现（确认单精度选项），封装 `Vqf` 与 `QuaternionEkf` 并列

## 外部 worktree 类布局重构

基线 `795e263`，分支 `refactor/class-layout`，目录 `../rmpp-class-layout`。当前按新的 SWD 观察布局重构：去成员尾下划线、展平实例配置、电机与超电状态展平、最小权限、声明与实现排序。infantry 跳过编译验证。代码与验证结论见 `tasks/10-global_refactor/README.md`，待用户审计与合并。
