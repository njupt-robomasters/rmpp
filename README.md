# RMPP

RoboMaster 机器人 STM32 控制程序，基于 CMake 构建，支持多兵种和多板卡

## 环境准备

| 工具 | 用途 |
| --- | --- |
| [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) | `arm-none-eabi-gcc` 交叉编译，需加入 PATH |
| CMake 3.22+、Ninja | 构建；CLion 自带，可直接使用 |
| OpenOCD 0.12+ | 下载与调试，配合 CMSIS-DAP 调试器；VS Code 的 EmberProbe 可一键安装 |
| Python 3 | 维护脚本 |

## 用 CLion 开发（推荐）

1. **打开工程**：打开仓库根目录，CLion 自动读取 `CMakePresets.json`
2. **选择应用**：`Settings › Build, Execution, Deployment › CMake` 中启用需要的配置，如 `infantry`、`examples-rc`
3. **构建**：工具栏选中配置和目标 `rmpp`，点击 Build
4. **下载调试**：新建 `OpenOCD Download & Run` 运行配置
   - Board config file：`boards/c_board/stm32f4discovery.cfg`
   - Executable：目标 `rmpp`
   - Run 下载运行，Debug 断点调试
5. **格式化**：`Settings › Editor › Code Style` 勾选 Enable ClangFormat，使用项目 `.clang-format`

## 用 VS Code 开发

推荐插件：**CMake Tools** 负责构建，[**EmberProbe**](https://marketplace.visualstudio.com/items?itemName=BakeSheep.emberprobe) 负责烧录、调试和实时变量观测

1. **构建**：命令面板执行 `CMake: Select Configure Preset` 选择应用，再执行 `CMake: Build`
2. **配置 EmberProbe**：打开侧边栏 EmberProbe
   - OpenOCD 环境：Windows 一键离线安装，或选择已有 OpenOCD 路径
   - MCU 配置 › 自动检测配置，确认 ELF 为 `build/<应用>/rmpp.elf`、目标 `stm32f4x.cfg`、调试器 `cmsis-dap.cfg`
3. **常用操作**

| 操作 | 作用 |
| --- | --- |
| 烧录 | 写入当前 ELF、校验并复位运行 |
| 复位调试 | 下载后复位，停在 `main` |
| 不复位调试 | 附加到正在运行的程序，保留现场，适合查卡死 |
| 实时读写 / 图表 | 运行中查看全局变量波形、在线改参、导出 CSV，适合调 PID |
| 外设寄存器 | 在“其他配置”下载 STM32F407 官方 SVD 后，暂停时查看 |

需要按 F5 启动时，在 `.vscode/launch.json` 中添加：

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "infantry",
            "type": "emberprobe",
            "request": "launch",
            "executable": "${workspaceFolder}/build/infantry/rmpp.elf",
            "runToEntryPoint": "main"
        }
    ]
}
```

> - 自动检测选工作区最新的 ELF，`build/` 下有多个应用时确认选中的是刚构建的那个
> - 断点调试需要 Arm GNU Toolchain 中的 `arm-none-eabi-gdb`；项目预设默认 Debug，变量可正常展开
> - 同一探针同时只能被一个 OpenOCD 占用，先停止 CLion 或其他软件的调试

## 用命令行编译与下载

```sh
# 查看可用应用
cmake --list-presets

# 配置并编译，产物为 build/infantry/rmpp.elf
cmake --preset infantry
cmake --build build/infantry

# 下载并复位运行
openocd -f boards/c_board/stm32f4discovery.cfg -c "program build/infantry/rmpp.elf verify reset exit"
```

## 目录结构

```text
rmpp/
├── app/          应用：各兵种和 examples 示例，入口为 app.cpp 中的 app_main()
├── lib/          公共算法与设备：电机、PID、IMU、遥控器、裁判系统等
├── boards/       板卡：CubeMX 工程 rmpp.ioc 与 BSP 外设实现
├── tools/        维护脚本
├── reference/    电机、开发板手册与比赛规则、通信协议
├── agents/       AI 的工作目录
└── build/        构建输出，每个应用一个子目录
```

## 常用操作

| 需求 | 位置或命令 |
| --- | --- |
| 车辆参数、遥控映射、操控手感 | `app/<应用>/` 下 `chassis.cpp`、`gimbal.cpp`、`shooter.cpp` 顶部常量 |
| 引脚、波特率等硬件配置 | `boards/<板卡>/rmpp.ioc`，外设实现在 `boards/<板卡>/bsp/` |
| 应用使用的板卡 | `app/<应用>/board.cmake` 中的 `RMPP_BOARD` |
| 新增应用后更新预设 | `python tools/update_presets.py` |
| 编译全部应用 | `python tools/check_builds.py`，日志在 `build/checks/` |
| 格式化代码 | `python tools/format_code.py [路径]`，加 `--check` 只检查 |

> cmake、ninja、clang-format 不在 PATH 时，脚本可用 `--cmake`、`--ninja`、`--clang-format` 指定 CLion 自带版本
