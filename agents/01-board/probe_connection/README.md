# EmberProbe 探针通信排查

2026-11-21，EmberProbe 0.9.3，Horco CMSIS-DAP v2，序列号 d8cd1ad9，VID:PID FAED:4873。

## 已验证

- 调试状态 none，采样 stopped，Get-Process 未发现 OpenOCD 进程
- MI_00 使用 Microsoft WinUSB；MI_01 提供 HID；当前默认 interface/cmsis-dap.cfg 未指定后端
- chip.read(debug) 在 openocd_start 阶段报 PROBE_COMMUNICATION_FAILED，USB read/write Entity not found，CMSIS-DAP CMD_INFO failed
- 用户重新插拔 USB 后 attachmentId 更新；再次 chip.read(debug) 出现相同错误
- 工作区固件、IOC、已安装扩展和内置 OpenOCD 文件均保持原样；未烧录、复位或暂停 MCU

## 修复结果

已将当前 OpenOCD 的 bin 和 openocd 目录复制到 agents/out/26-probe_connection/openocd-hid，新增 interface/cmsis-dap-hid.cfg，显式选择 HID；原始脚本保留。

1. 用户已在 EmberProbe 侧栏将 OpenOCD 路径设置为本任务 agents/out/26-probe_connection/openocd-hid/bin/openocd.exe 的绝对路径（原值 openocd）
2. Agent 已经 config_set 设置 debugger=cmsis-dap-hid.cfg（原值 cmsis-dap.cfg）
3. chip_read(debug) 成功：probeName=CMSIS-DAP、probeVersion=v208、transport=SWD、clock=1800 kHz、targetName=stm32h7x.cpu0

当前通过 HID 恢复探针与目标的调试通信，USB bulk 失败的底层原因仍待单独排查；尚未执行烧录验证。

用户随后要求切回 jelly_sentry，已通过 config_set 将 EmberProbe ELF 设为 build/jelly_sentry/rmpp.elf；该构建目录缓存确认 RMPP_APP=jelly_sentry、CMAKE_BUILD_TYPE=Debug。当前会话缺少 VS Code 命令执行和桌面控制工具，CMake Tools 的界面激活预设仍需用户选择 jelly_sentry。本轮未重新编译或烧录。

当前配置依赖 agents/out/26-probe_connection/openocd-hid 目录，使用期间保留该目录。HID 脚本源文件另存于本任务 cmsis-dap-hid.cfg，便于重建。回退时经 config_set 恢复 debugger=cmsis-dap.cfg，用户将 OpenOCD 路径恢复为 openocd。

OpenOCD 官方后端说明：https://www.openocd.org/doc/html/Debug-Adapter-Configuration.html
