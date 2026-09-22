RMPP：STM32 机器人控制程序

快速开始
1. 安装 Arm GNU Toolchain、CMake 3.22+、Ninja，加入 PATH；更新预设需要 Python 3。
2. 在根目录执行：
   cmake --preset hero-debug
   cmake --build --preset hero-debug
3. 固件：build/hero-debug/rmpp.elf；业务入口：app/hero/app.cpp 的 app_main()。
4. 用 cmake --list-presets 查看应用；可选 hero、infantry、sentry 及九个 examples，每个均有 debug/release。

CLion：打开根目录，使用项目 CMake Presets，选中所需应用的配置后构建。
格式化启用项目 .clang-format；命令行使用同一 clang-format 运行 tools/format_code.py。
板卡调试配置：boards/stm32f407_igh6/stm32f4discovery.cfg。

常用修改
- 业务逻辑和车辆参数：app/<应用>/。
- 公共算法及设备逻辑：lib/。
- 引脚、波特率等硬件参数：板卡 .ioc；外设实现和启动：板卡 bsp/。
- 新增应用后执行：python tools/update_presets.py。
- 完整编译检查：python tools/check_builds.py。

外设用法：auto can = bsp.can(1);
也可直接构造：Can can(1);
发送：can.send({.id = 0x123, .data = {1, 2}, .size = 2});
接收：can.add_rx_callback([](const CanFrame& frame) { /* 处理中断报文 */ });

目录、接口及新板卡接入：docs/architecture.md
验证结果与人工审计入口：docs/verification.md
数值单位和历史问题：docs/unit-migration.md
