RMPP：STM32 机器人控制程序

快速开始
1. 安装 Arm GNU Toolchain、CMake 3.22+、Ninja，加入 PATH；更新预设需要 Python 3。
2. 在根目录执行：
   cmake --preset hero
   cmake --build build/hero
3. 固件：build/hero/rmpp.elf；业务入口：app/hero/app.cpp 的 app_main()。
4. 用 cmake --list-presets 查看应用；可选 hero、infantry、sentry 及九个 examples，每个默认使用 Debug，名称不带构建类型后缀。

CLion：打开根目录，使用项目 CMake Presets，选中所需应用的配置后构建。
格式化启用项目 .clang-format；命令行使用同一 clang-format 运行 tools/format_code.py。
板卡调试配置：boards/stm32f407_igh6/stm32f4discovery.cfg。

常用修改
- 业务逻辑和车辆参数：app/<应用>/；遥控器映射、操控手感、弹速弹频及按键调整步长在 chassis.cpp、gimbal.cpp、shooter.cpp 顶部常量中。
- 公共算法及设备逻辑：lib/。
- 引脚、波特率等硬件参数：板卡 .ioc；外设实现和启动：板卡 bsp/。
- 新增应用后执行：python tools/update_presets.py。
- 完整编译检查：python tools/check_builds.py；cmake、ninja 不在 PATH 时用 --cmake、--ninja 指定 CLion 自带版本。
- 格式化：python tools/format_code.py [路径]；用 --clang-format 指定 CLion 自带的 clang-format。
- CMake 结构、依赖关系图与验证说明：docs/build.md。
