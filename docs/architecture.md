# 多板卡设计约定

用途：说明多板卡迁移的最终接口、目录与构建方式，供实现和人工审计使用。
状态：方案已讨论，尚未迁移代码；当前构建步骤见根目录 `../README.`。

## 目录与职责

```text
CMakeLists.txt / CMakePresets.json  根入口，选择 app 及 Debug/Release 配置
app/<name>/CMakeLists.txt          app 源码、默认板卡、工具链与依赖组合
lib/CMakeLists.txt                 扫描并编译公共库
lib/bsp/                          无 HAL 依赖的公共外设声明
boards/<board>/CMakeLists.txt      CubeMX 默认生成工程
boards/<board>/bsp/                板卡实现、bsp.hpp、手写 CMake 与工具链适配
tools/update_presets.py            扫描 app，生成各 app 的 Debug/Release 预设
```

原 `rmpp` 内容迁入根目录的 app、lib 等目录；生成工程归所属板卡。
构建缓存重新生成；IDE 配置按新入口调整。板卡名称区分实际硬件及配置。

## 入口和外设

CubeMX 的 `main()` 在 `MX_*_Init()` 完成后调用：

```c
bsp_init();
app_main();
```

`void bsp_init()` 完全由板卡 BSP 实现，配置、启动外设及接收通知；外设通过中断持续工作。`void app_main()` 由应用实现，管理业务循环。

板卡 `bsp.hpp` 用 `#ifdef __cplusplus` 和 `extern "C"` 声明两个入口，将其余 C++ 内容放在条件编译区域内，因此 `main.c` 也可包含它。

```cpp
auto can = board.can(1);
can.add_rx_callback([this](const CanFrame& frame) { receive(frame); });
can.send(frame);
```

- Board 是无状态访问入口，通道状态通过函数内静态对象创建。全局变量可获取句柄、注册回调；构造只保存数据，硬件操作在 HAL 初始化之后执行。
- ID 使用 int，BSP 用 switch 显式映射，允许不连续。`Can` 按值复制，共享同一通道状态；公共接口没有 CanDriver 层。
- 不存在的 ID 返回空句柄；`bool(can)` 判断资源是否存在，send/注册返回 false，无硬件行为。
- send 只接收一个报文结构体；邮箱满返回 false，避免无限等待。成功表示提交给控制器。
- 经典 CAN 报文用固定 8 字节 `std::array` 和实际字节数；array 长度表示容量，容器自身不分配堆内存。CAN FD 需要另行扩展报文能力。

## 回调

- 名称为 `add_rx_callback(callback)`，单参数 `std::function<void(const CanFrame&)>`，支持捕获 lambda。
- 每个通道用 vector 保存全部回调；接收中断内按注册顺序调用，不按报文 ID 筛选注册方。应用自行判断报文内容，重复注册会重复调用。
- 注册可在外设启动前后进行。当前单核裸机实现修改 vector 时保存并屏蔽中断，结束后恢复原状态；注册发生在普通代码中，回调执行中不修改列表。
- vector 扩容及 std::function 可能分配内存，注册是低频操作；当前关闭异常，内存不足不承诺返回 false。接收分发只遍历列表。
- 回调短小、非阻塞，引用的报文仅在调用期间有效；捕获对象在订阅期间保持存活且地址稳定。
- CAN 使用 FIFO 接收中断；UART 可使用空闲中断。未注册时收到报文直接消费，不另建缓存或要求 poll。

## CMake 组合

每次配置选择一个 app 和板卡，使用独立构建目录。脚本扫描 `app/*/CMakeLists.txt`，生成 `<app>-debug`、`<app>-release` 的 configure/build presets；根入口自动转入所选 app。

板卡配置合并到 app 的 `CMakeLists.txt`。为让工具链在首次 `project()` 前确定，app 文件先设置默认板卡和工具链，再定义一个构建函数，由根入口调用：

```cmake
# 根 CMakeLists.txt 的结构示意
cmake_minimum_required(VERSION 3.22)
set(RMPP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(RMPP_APP "hero" CACHE STRING "Application")
include("${RMPP_ROOT}/app/${RMPP_APP}/CMakeLists.txt")
project(rmpp_workspace LANGUAGES C CXX ASM)
rmpp_add_app()
```

app 文件在函数外设置 `RMPP_BOARD` 和 `CMAKE_TOOLCHAIN_FILE`，通过 `CMAKE_CURRENT_LIST_DIR` 保存自己的目录；`rmpp_add_app()` 内添加 lib、板卡 BSP 和应用源码，链接到生成的固件目标。这是一次读取、一个构建函数，不拆 target.cmake。

```cmake
# 各源码目录按同一规则收集文件
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
```

`CMAKE_CURRENT_SOURCE_DIR` 是当前 add_subdirectory 的源码目录；include 不改变它。
因此 include 进来的 app 文件用保存的 app 目录扫描自身；普通 lib/BSP 子目录使用上述表达式即可。构建目录位于源码扫描范围之外。

`CACHE STRING` 定义可通过 `-D` 或 preset 覆盖的字符串配置，保存在构建目录中；源码默认值不覆盖已有缓存。切换板卡使用新的构建目录。

板卡 BSP 用 add_subdirectory 添加上一级 CubeMX 工程，然后定义自身对象库。当前 stm32cubemx 接口目标提供 HAL 头文件和芯片宏；lib 编译公共实现，app 和 BSP 使用对象库，最终链接到 CubeMX 已创建的固件目标。保留生成工程打印，`cmake --build --preset <name> --verbose` 查看完整命令。

当前工程迁移时的适配点：

- 根生成 CMake 中已有手写的 `add_subdirectory(rmpp)`，迁出这部分业务接入逻辑；默认生成部分继续归 CubeMX 管理。
- 生成工具链依赖旧根目录定位链接脚本；BSP 手写适配修正路径，所有目标使用同一套 CPU/FPU/ABI 参数。
- 最终链接使用 C++ 链接器；补充链接时兼容生成文件当前的无关键字 target_link_libraries 写法。
- DWT、FlashDB 硬件适配等板卡实现随 BSP 迁移，公共 lib 只保留统一声明和通用实现。

## 实施与审计

实现保持可人工逐段阅读，验证关注实际固件的 Debug/Release 构建、接口行为及回调修改与中断并发。发现问题修正简单实现；完成后说明改动和测试结果，用户审计后决定上车测试。
