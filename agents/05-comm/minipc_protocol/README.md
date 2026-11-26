# 自瞄与导航协议重构

状态：已并入 master，待用户审计与上车；导航上行目标坐标未接入

2026-11-24 用户授权在当前工作区实施。对接文档见根目录 `docs/minipc_protocol.md`，模块说明见 `agents/05-comm/minipc.md`。

## 用户要求

- 帧头改为 1 字节，不再重复
- CRC 校验覆盖帧头
- 自瞄上行：红方标志、弹速、四元数
- 自瞄下行：识别标志、开火标志、yaw、pitch、距离
- 导航下行：过洞标志、vx、vy、yaw、x、y（当前坐标）
- 导航上行：大 yaw 编码器角、目标 x、y 坐标

## 已确认决定

- 四种帧各用一个帧头：`A1` 自瞄下行、`A2` 自瞄上行、`A3` 导航下行、`A4` 导航上行，不加命令码
- 四元数字段顺序 `x y z w`，为机体到世界旋转，取自 `attitude.q`
- 导航上行 yaw 为大 yaw 相对底盘的编码器实测角，下行 yaw 为相对大 yaw 当前朝向的期望转角，两向均用 rad。x/y 坐标用 m，取上位机自定位场地系
- `action_flag` 删除，改为过洞标志 `is_through_hole`；置 1 时底盘随头对准枪口、平移照常，置 0 时小陀螺
- 自瞄下行 yaw 俯视逆时针为正、pitch 向下为正；内部 ZYX 右手系同号，删除原先的 pitch 取负

## 改动

- `lib/minipc/aim`：`RxFrame` 17 字节、`TxFrame` 24 字节，帧头常量 `SOF_RX`/`SOF_TX`；`set_imu()` 改为 `set_attitude(const Quaternionf&)` 直接写 x y z w；CRC 覆盖帧头；接收改为 1 字节帧头比对
- `lib/minipc/nav`：`RxFrame` 24 字节（过洞标志、vx、vy、相对转角、x、y）；`TxFrame` 15 字节，`set_target(x, y)` 设置目标坐标，`send(yaw_ecd)` 填本次编码器角并发送；`action_flag` 改为 `is_through_hole`，新增公开 `x`、`y`；原始帧统一为 `raw_rx`、`raw_tx`
- `app/jelly_sentry`、`app/steer_sentry`：`aim.set_attitude(attitude.q)`；`control_by_auto()` 按过洞标志选择随头或小陀螺，两个分支对调
- `agents/05-comm/nav`：`aim_test.cpp`、`nav_test.cpp`、`support.hpp` 换用新帧，补导航上行发送与 x/y 字段检查

## 复现

主机测试（项目根目录）：

```powershell
New-Item -ItemType Directory -Force agents/tmp/minipc_protocol
g++ -std=c++23 -O2 -Wall -Wextra -Werror -DEIGEN_DONT_VECTORIZE -DEIGEN_MAX_ALIGN_BYTES=0 -Ilib -isystem lib/third_party/eigen-3.4.0 agents/05-comm/nav/aim_test.cpp lib/minipc/aim.cpp -o agents/tmp/minipc_protocol/aim_test.exe
& agents/tmp/minipc_protocol/aim_test.exe
g++ -std=c++23 -O1 -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-function -DEIGEN_DONT_VECTORIZE -DEIGEN_MAX_ALIGN_BYTES=0 -Iagents/05-comm/nav -Ilib -isystem lib/third_party/eigen-3.4.0 agents/05-comm/nav/nav_test.cpp lib/minipc/nav.cpp lib/robot/gimbal_fold.cpp lib/motor/motor.cpp lib/pid/pid.cpp -o agents/tmp/minipc_protocol/nav_test.exe
& agents/tmp/minipc_protocol/nav_test.exe
```

ARM 对象编译（用已有 `build/<目标>/compile_commands.json`）：

```powershell
python agents/05-comm/minipc_protocol/compile_arm.py
```

格式检查：

```powershell
python -m pip install --target agents/tmp/minipc_protocol/tools clang-format==22.1.0
python tools/format_code.py --clang-format "E:/rm/code/potential/rmpp/agents/tmp/minipc_protocol/tools/clang_format/data/bin/clang-format.exe" --check lib/minipc app/jelly_sentry app/steer_sentry
```

## 验证结果

- Aim 协议测试通过：新帧布局与 CRC、全部拆分位置、粘包与重叠帧头、每个位置缺字节恢复、CRC 错误、非有限角度、超时清目标，以及上行 `x y z w` 与红方/弹速原值发送
- Nav 与 pitch1 共 90,961 项检查通过：新帧字段、导航上行黄金帧、全部分片、插/丢字节恢复、噪声恢复、超时清零、第二实例隔离
- 两个哨兵的 `lib/minipc/aim.cpp`、`nav.cpp` 与 app、chassis 共 8 个 ARM 对象零警告
- clang-format 22.1.0 检查 9 个手写文件通过
- 未烧录、复位或改写硬件；`15-minipc_nav/chassis_test.cpp` 的桩与当前应用头文件冲突，本轮之前已失效，未修
- 2026-11-25 大 yaw 协议更新：Nav 与 pitch1 共 90,968 项检查通过，覆盖 15 字节上行参考帧、重复接收 `+π/2` 和回复 `0`；对接文档的两条导航参考帧逐字段及 CRC 核对通过。两个哨兵的 `nav.cpp` ARM 对象零警告，Nav 源码格式检查通过

## 遗留

- 导航上行目标坐标来源未接入板侧，哨兵应用尚未调用 `Nav::send(yaw_ecd)`。候选来源为裁判 0x0303 选手端小地图指令（`Referee::map_command`，当前为私有成员），需确认与上位机自定位同系后再接
- 上位机侧联调用 `docs/minipc_protocol.md` 的参考帧

## 轮腿上板接入

- `leg_up` 复用 CDC1 自瞄协议，沿用 A1/A2 帧。`minipc_loop()` 填充上行并检查断联，`app.cpp` 以 200 Hz 调用 `aim.send()`。云台和底盘各自判断 `vt13.is_aim && aim.is_detect`，用于视觉跟踪或清零随头的人工 yaw 速度前馈。小陀螺仍使用手动角速度
- 自瞄模式仅在 `is_detect && is_fire` 时放行原 VT13 弹频；手动发射输入、原有使能和热量保护仍生效。目标丢失或 100 ms 无有效帧时恢复人工云台控制
- `python agents/05-comm/minipc_protocol/verify_leg_up.py` 通过：用实际 app 循环和协议解析，在主机注入下行帧，检查三标志的八种组合、方向、人工发射输入、超时恢复、跟随前馈、热量保护和 200 Hz 上行内容
- `python agents/04-leg/chassis/test_leg_up_control.py`、Aim 协议主机回归、CLion clang-format 检查及 `leg-leg_up` 六个应用 ARM 对象编译通过；未完整链接、烧录或上车
