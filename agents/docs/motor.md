# 电机库开发约定

仅在修改 `lib/motor/**` 时读取本文档

## 构造与配置

- 各层使用平面的 Motor::Config 构造输入，按通信、量程与减速比、安装方向与零点分组
- 实例直接保存字段，删除 bus/drive/install/raw 结构层；型号 MAX_CURRENT、MAX_TORQUE、REDUCTION 传入 Motor 构造函数补齐零值
- VESC 无固定型号，app 显式给出 max_current、max_torque、reduction；摩擦轮按 M3508 常量
- 固定量程和硬件绑定加 const；max_current、max_torque 供机构计算，offset 供零点存储，外部只读；offset 经 set_offset()/set_zero() 修改
- 不保存 kt，电流与力矩按 max_torque/max_current 换算；限流交给 PID max_out
- 减速比与型号不同时同时给出 max_torque；自行发送的协议使用 tx_id/rx_id，DJI 组帧只需 rx_id
- 原始反馈为 protected raw_current/raw_torque/raw_speed/raw_position；仅供 SWD 的温度等诊断字段使用 private
- Config 仅用于构造输入，实例无 Config 成员；布局见 conventions.md

## 基类与子类

- 适配父类重构时优先修改子类
- 父类只保留通用控制和反馈修正
- 最终型号负责注册回调、筛帧和解析型号特有字段
- DJI 型号解析反馈后直接调用 `Motor::can_callback()` 完成通用反馈修正
- 其他协议子类解析后写入原始标量，再调用 `Motor::can_callback()` 修正反馈并刷新在线状态
- 周期 `update()` 只完成离线检测，闭环由机构库完成

## 协议刻度

- 仅在一处使用的缩放刻度直接写在换算处
- 不增加 `feedback_scale`、`command_scale` 等单用途配置字段

## 零值与限幅

- 量程与减速比零值字段按型号常量补齐
- `set_current()` 先将非有限值置零，再按 `max_current` 限幅并换算转矩
- `set_torque()` 先将非有限值置零，再按 `max_torque` 限幅并换算电流
- 反馈解析中有符号的转速、电流先转 `int16_t` 保留反转符号
- `raw_speed` 为电机侧转速，只由 `Motor::can_callback()` 除一次减速比

## DJI 组帧

- app 创建长度为 8 的零初始化组帧，调用各电机 `write_can_frame()` 填入对应两字节后统一发送
- 型号根据控制帧 ID 和反馈 ID 校验分组、选择协议刻度并计算槽位
- 无返回值，校验失败时不修改帧；离线或未使能时成功写入零命令

## GM6020 电压帧

- 电压帧 0x1FF/0x2FF 把 `current_ref` 按 3 A ↔ 25 V 换算为电压，电流帧 0x1FE/0x2FE 直接发电流
- 电流与力矩按 3 A ↔ 2.223 N·m 换算，与其他型号一致
- 反电动势前馈系数为构造第二参数 `float ke = 0`，V/(rad/s)，发送时叠加 `ke·speed` 并限幅 ±25 V；0 不补偿，需要时传手册值 `GM6020::KE`（1 V/13.33 rpm）或按比例缩小，如 `0.5f * GM6020::KE`
- 可选前馈系数是“配置不给默认实参”约定的例外

## RMD-L-7025

- 直连脉塔协议，`tx_id = 0x140 + ID`，`rx_id = 0x240 + ID`，不再使用 DJI 格式转接板
- `send()` 用内部计数器轮流发送转矩闭环 0xA1（应答温度、电流、转速）和读取单圈角度 0x94（应答角度）
- 电机应答上限 500 Hz，单电机调用周期不小于 2 ms，各报文有效频率为调用频率的一半
- 离线或未使能时 0xA1 发零电流，用应答维持在线检测

## X630（N630/P630）

- CAN 协议、刻度与组帧同 M3508（C620），物理常数暂按 M3508 取值；无 `send_quick_set_id()`
- HFI 静音：构造第二参数 `Mute`，重写 `set_current/set_torque`，设置后滞回判断，静音时 `current_ref/torque_ref` 置零
- 目标电流 < `enter_current`（1 A）且转速 < `enter_speed`（10 rpm）进入静音，目标电流 > `exit_current`（2 A）才退出
