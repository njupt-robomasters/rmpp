# 应用组织约定

仅在修改 `app/**` 时读取本文档

## 职责边界

- app 选择硬件和车型策略
- lib 接收句柄或引用
- 板卡 BSP 实现外设
- 校准组合键、断联保持使能等车型策略在 app 展开
- lib 不为车型策略保留开关或流程

## 启动与循环

- CubeMX 初始化后依次执行 `bsp_init()`、`app_main()`
- `app.cpp` 分为 `setup()`、`loop()`、`app_main()`：`setup()` 放一次性初始化；`loop()` 依次轮询各设备与模块；`app_main()` 先调用 `setup()`，再用 `static Timer app_timer` 按 1 kHz 调度 `loop()`，并用 `static Monitor app_monitor` 统计频率与占用（SWD 观察）
- 低于 1 kHz 的任务在 `loop()` 内用 `static Timer <用途>_timer` 分频后提前返回，如 cdc 示例的 `print_timer`
- dummy 用 `uint32_t` 计数递增、取余分频，无手动清零：yaw 每 2 次循环发送，超电每 10 次循环发送，模拟裁判每 100 次循环结算
- 使用普通 loop，不引入操作系统或整车包装类

## 文件组织

- 公共内容放在 `app.hpp`
- 各模块在同名 `.hpp` 声明、`.cpp` 实现
- 文件名不加 `_loop` 后缀
- 配置和实例跟随所属模块
- 组件可独立使用
- 整合管理器引用外部实例

## 遥控器映射

- 不设置遥控器汇总层
- 底盘、云台、发射分别在对应 `.cpp` 直接读取 VT13、FSi6X
- 操控参数和跨周期状态使用文件内 `static`
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`
