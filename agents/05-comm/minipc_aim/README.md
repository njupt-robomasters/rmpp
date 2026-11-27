# Aim 自瞄双向通信

本目录保留21/18字节自瞄协议的历史测试，适用基线8280d88。当前生产协议测试见 `../nav/` 和 `../minipc_protocol/`。

2026-11-20 用户授权在当前工作区实现，并明确期望 yaw/pitch 指向最终安装 IMU 的头部。当前协议与坐标说明见 `../minipc.md` 的 Aim 部分。

## 行为

- CDC1：上行 A5 5A 共 21 字节，5 ms 周期；下行 A5 5B 共 18 字节，收到有效新帧后发布
- `Aim(Cdc(1), {.timeout = 200 * ms})`；Config::timeout 默认 200 ms，始终按有效帧的接收时间计时
- `set_referee(is_red, bullet_speed)` 设置颜色与裁判实测弹速；`update(yaw,pitch,roll)` 接收头部 IMU 弧度，上传时转度，线上 pitch 上正与内部右手 ZYX pitch 反号
- 视觉目标角换回内部弧度后，经已有 gimbal_loop 送入 GimbalFold 的 IMU_MODE 总目标，大小 yaw 与 pitch2 分配控制；pitch1 仍固定失能
- 无目标有效帧立即清空目标/开火许可，保持连接在线；坏帧不刷新超时，超时清空 vision 和连接状态
- Aim/Nav 共用 protocol.hpp 的 MODBUS CRC 与小端 float32 编解码，导航行为保持原有约定
- 原 imu 公开写入改为 update 参数，原裁判结构写入改为 set_referee，比赛进度和剩余时间占位输入已移除

## 复现与结果

```powershell
python agents/05-comm/minipc_aim/verify.py
```

使用已有 `agents/.tools/ziglang/zig.exe`，无需再安装测试工具。脚本结束后删除本任务 Zig 全局/本地缓存，保留小体积日志、测试程序和 ARM 对象。

- Aim 主机检查 4,262 项通过：上行字段布局、CRC、200 Hz 调度、发送忙恢复、最新姿态与弹速、角度换算；首帧逐字节、所有切分点、粘包、插字节/丢字节后的重同步、坏 CRC、NaN/Inf、非法布尔、负距离、丢目标撤销开火、默认 200 ms 与自定义 20 ms 超时、重连、CDC1/2 隔离和 2000 组噪声
- Nav 与 pitch1 回归 90,578 项通过，共用编解码后的导航接收和真实 DM4310 失能帧行为保持通过
- 13 个 ARM 对象编译通过，零警告：jelly_sentry 六个应用源文件、Aim/Nav，以及 sentry 五个调用源文件
- 当前仅保留 build/jelly_sentry 的编译数据库。jelly_sentry 对象使用真实 mc02 配置；sentry 调用适配使用同一 ARM C++ 环境做接口编译，C 板专属构建待恢复其配置后执行
- 主机兼容警告来自第三方 Eigen 与 libc++ 的弃用浮点元信息，未修改第三方库
- 格式检查限定本轮 9 个手写源码文件；未完整链接或操作硬件。上板时核对视觉与头部 IMU 的世界零点、俯仰上正及 roll 观察方向、实际帧率与超时开火撤销

编译与测试日志、ARM 编译摘要生成在 `agents/out/23-minipc_aim/`，可按需清理后重跑。

## EmberProbe 只读实机观察（2026-11-20）

- 配置 ELF 为 build/jelly_sentry/rmpp.elf，SHA-256 `d5df19300a9fc5f541906077a4d07c8ff0f1206c3d6137cc13859d1cb4863001`，Bridge 返回 firmwareConsistency=match，烧录记录为当日 21:00:41（香港时间）
- 10 次采样中 aim.is_connect_vision=0、aim.callback_timer.last_cycles 的精确值始终为 `0`；CDC1 接收回调低通频率约 19.87–20.02 Hz
- CDC1 RX 缓冲前 18 字节始终为 `A5 5A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 AB 01`，14 字节零载荷的 MODBUS CRC=0x01AB 正确；当前视觉下行帧头与约定 A5 5B 不匹配，Aim 跳过该帧
- 上位机应将该帧第二字节改为 5B；CRC 不含帧头，当前全零载荷的尾部 AB 01 保持。修正后空目标帧应令通信在线、is_detect/is_fire 保持 false
- CDC1 TX 缓冲持续更新，10 份 21 字节快照的 CRC 全部通过，采样点 is_tx_busy=0；send_timer.dt 多为 5.002–5.010 ms，其中一次约 5.997 ms。视觉端实际收到的上行帧率需由其接收记录核对
- 本次仅临时只读变量采样，未烧录、复位、halt、改写目标内存或改变代码协议

### 上位机修正帧头后复查

- 20 次主采样和 10 次补充采样中，aim.is_connect_vision/has_vision 保持 1，有效接收 last_cycles 持续递增；修正后的 RX 帧头为 A5 5B，20 份 RX 快照 CRC 全部通过，接收回调频率约 32.83–33.33 Hz
- is_detect=1，目标持续更新；样本目标 yaw 约 0.665–0.675 rad，内部 pitch 约 -0.180–-0.171 rad，距离多数约 0.68–0.70 m，部分样本为 0
- MCU 上行缓冲持续更新，发送调度约 5–6 ms。初次 20 份 TX 快照 19 份通过 CRC，1 份不一致；随后复查 10 份全部通过。运行态内存读取与持续更新缓冲存在跨帧采样的可能，线上上行是否存在 CRC 错误需视觉端接收统计核实
- 复查全程临时只读采样，未更改 MCU 运行状态、内存、固件或协议
