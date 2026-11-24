# 协议解析与 minipc/rc/referee 风格统一

2026-11-24 用户授权在当前工作区实施。统一写法见 `../../docs/conventions.md`“协议接收”。

## 改动

- Nav、Aim、VT13、Referee 接收回调统一为同一套步骤：帧头不符丢 1 字节、不足整帧等待、校验不符丢 1 字节、有效帧整帧出队后保存原始帧并解析；接收缓冲统一命名 `rx_fifo`，计时器统一 `callback_timer`
- Nav：删除 `has_command`、浮点有限性与 flag 范围检查、回调内超时判断和 `IrqLock` 快照；`command` 展开为 `vx`、`vy`、`yaw`、`action_flag`；原始帧 `raw`
- Aim：原始收发帧 `raw_rx`、`raw_tx`；`set_imu()`、`set_referee()` 直接写 `raw_tx`，`send()` 补 CRC；字段原样映射，删除数值检查
- VT13：解析内联回 `callback()`，删除 `parse_rc_frame()`、`parse_referee_frame()`；`deadline` 改名 `dead_zone`，`get_joy_stick` 改名 `get_joystick`；修正 crc16 字段注释
- Referee：`receive()` 改名 `callback()`，删除 `receive_timer` 的 100 ms 空闲清缓冲；`connect_timer` 改名 `callback_timer`；`parse()` 用 `read()` 模板合并定长报文分支，按命令码升序；删除拷贝禁用
- CRC：`CRC8`/`CRC16` 静态类改为自由函数 `crc8_referee`、`crc16_referee`，删除 `verify()`；`crc16_modbus.hpp` 补头文件
- FSi6X：`deadline` 改名 `dead_zone`，回调参数改 `size_t`，解析拆到 `parse_raw()`，`update()` 不再重启计时；修正 ch0/ch1 注释
- Referee、RefereeSender、RobotUi、FSi6X 的 `Config` 统一为 `} const config;`；位运算字面量补 `u`

## 复现

```powershell
python agents/tasks/20-protocol_unify/compile_arm.py
python agents/tasks/12-vt13_control/test_vt13.py
python agents/tasks/13-referee_relay/test_fifo.py
python agents/tasks/13-referee_relay/test_referee_relay.py
python agents/tasks/13-referee_relay/test_referee_sender.py
```

Nav、Aim 主机测试命令见 `../15-minipc_nav/README.md`，本轮用主机 g++ 编译。

## 验证结果

- Aim 主机测试通过；Nav/pitch1 90,583 项检查通过；VT13、Fifo、RefereeRelay、RefereeSender 主机测试通过
- 41 个 ARM 对象零警告：两台哨兵、dummy、leg_up、leg_down、examples/rc、examples/referee 的 app 文件，以及 C 板与 mc02 各一遍的 9 个库文件
- CLion 随附 clang-format 22 检查通过
- `15-minipc_nav/chassis_test.cpp` 的桩与当前 jelly_sentry 头文件冲突（重复定义 ChassisMatrix、VT13 无 `vx`），本轮之前已失效，未修
- 未烧录、复位或改写硬件
