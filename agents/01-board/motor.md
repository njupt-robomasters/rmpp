# 电机库开发约定

仅在修改 `lib/motor/**` 时读取本文档

## 构造与配置

- 各层使用平面的 Motor::Config 构造输入，按通信、量程与减速比、安装方向与零点分组
- 实例按值保存一份公开的 `Config config`，通信、超时、量程、减速比、方向和零点直接读取 config，不再逐项保存同名成员；型号 MAX_CURRENT、MAX_TORQUE、REDUCTION 传入 Motor 构造函数补齐 config 中的零值
- VESC 无固定型号，app 显式给出 max_current、max_torque、reduction；摩擦轮按 M3508 常量
- 型号常量按当前调用保留权限：M3508 的三项物理常量、GM6020 的 MAX_CURRENT/KE、RMDL7025 的 MAX_CURRENT 供外部使用，其余型号常量为 private；构造绑定和 set 输入位于反馈前，多圈历史状态与计时器位于反馈后
- 外部经 `motor.config.max_current/max_torque` 读取量程，经 `motor.config.offset` 维护和存储零点；其余配置在构造后保持固定，CAN 回调仍注册在构造时的总线上
- 不保存 kt，电流与力矩按 max_torque/max_current 换算；限流交给 PID max_out
- 减速比与型号不同时同时给出 max_torque；自行发送的协议使用 tx_id/rx_id，DJI 组帧只需 rx_id
- 子类把电机侧原始反馈装入 protected `Motor::Feedback` 传给 `can_callback()`，不保存 raw 电流、力矩、转速；基类仅以 private `raw_angle`、`raw_angle_multi` 维护多圈累计；仅供 SWD 的温度等诊断字段使用 private
- 电机库不提供 `set_offset()`、`set_zero()`；校零策略由 app 维护，布局见 conventions.md

## 角度与零点

- 电机旋转量统一命名 angle：输出轴单圈反馈 `angle`（[-π, π)）、连续反馈 `angle_multi`，单位 rad；VESC 的角度设置接口为 `set_angle()`，云台角度 PID 使用 `*_angle_pid*`
- 首个有效角度帧以原始编码器角度初始化多圈累计，后续按最短角差累加；相邻有效角度采样的位移须小于半圈，累计保留到实例销毁
- 只有 `Feedback.angle` 有值时推进多圈初始化和累计；RMD 0xA1 应答、VESC 状态帧不传 angle
- `angle_multi = dir * (raw_angle_multi - config.offset) / config.reduction`；offset 为电机侧弧度，方向和减速比只在输出时修正
- 减速比为 1 时单圈反馈直接修正 raw_angle，其他减速比将 angle_multi 归一化；外部修改 offset 后，两种反馈在下一次回调刷新，累计原始角度保持连续
- 外部校零使用 `config.offset += dir * angle_multi * config.reduction`；update 检测超时后清零 angle，保留 angle_multi 和原始累计，恢复回调后重新计算角度并接续累计

## 基类与子类

- 适配父类重构时优先修改子类
- 父类只保留通用控制和反馈修正
- 最终型号负责注册回调、筛帧和解析型号特有字段
- 子类解析后调用 `Motor::can_callback({...})`，用指定成员初始化写出本帧携带的 current、torque、speed、angle（电机侧，SI 单位），未携带的量留空
- 基类每次回调刷新在线状态；有值的量修正方向和减速比后更新，留空的量保持上次反馈；角度输出每次按当前 offset 重算
- 电流与力矩只传其一时由基类按 max_current/max_torque 换算另一项，子类不做换算；DM 传 torque，其余型号传 current
- 帧内不含任何反馈的应答（如 VESC STATUS_2~6）传 `{}`，只刷新在线状态
- 周期 `update()` 只完成离线检测，闭环由机构库完成

## 协议刻度

- 仅在一处使用的缩放刻度直接写在换算处
- 不增加 `feedback_scale`、`command_scale` 等单用途配置字段

## 零值与限幅

- `set_output()` 返回 void，按协议原生输出设置：基类调用虚函数 `set_current()`（A），DM 系列和 EL05 覆盖为 `set_torque()`（N·m）；保留显式电流、力矩接口
- RMDL7025 的 0xA1 输入为电流；GM6020 默认输出仍为电流，电压帧沿用发送层换算；VESC 命令更新与 X630 静音继续经原设置接口执行
- 使用默认输出的机构由 app 配置 PID 输出单位和限幅，电机设置函数保留量程限幅
- 量程与减速比零值字段按型号常量补齐
- `set_current()` 先将非有限值置零，再按 `max_current` 限幅并换算转矩
- `set_torque()` 先将非有限值置零，再按 `max_torque` 限幅并换算电流
- 反馈解析中有符号的转速、电流先转 `int16_t` 保留反转符号
- `Feedback.speed` 为电机侧转速，只由 `Motor::can_callback()` 除一次减速比

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

## EL05 私有协议

- 使用上游平铺 `Motor::Config`，解析后经 `Motor::Feedback` 传入torque/speed/angle，由基类按 `max_current/max_torque` 换算电流；当前app显式使用5 Nm与 `5/0.94` A保留实测比例
- 参考 Joushua233/rmpp 的 old@e7bbcb7 中 EL05 和 sentry/pitch，使用 1 Mbps 经典扩展帧，8 字节；tx_id 为电机节点、rx_id 为主机节点，二者均为 8 位，电机侧 protocol_1=0
- 手册印刷页 30 的通信类型 2：扩展 ID bit28~24 为类型2，bit23~22 模式、bit21~16 六项故障、bit15~8 电机节点、bit7~0 主机节点；Byte0~1 角度、Byte2~3 速度、Byte4~5 力矩均为16位大端，Byte6~7 为温度0.1℃
- 运控类型1把力矩编码到 ID bit23~8；数据区 p/v/kp/kd 置零，主控闭环输出力矩。使能为类型3、停止为类型4，ID bit15~8 为主机节点、bit7~0 为电机节点；停止帧 Byte0=1 清错
- 故障期间停止出力，每500ms自动尝试清错，收到fault=0反馈后且上层仍使能才恢复；clear_fault_count记录尝试次数，不代表成功恢复
- 角度 ±12.57 rad、速度 ±50 rad/s、力矩 ±6 Nm，均按65535量化；应用限幅独立于协议量程

## X630（N630/P630）

- CAN 协议、刻度与组帧同 M3508（C620），物理常数暂按 M3508 取值；无 `send_quick_set_id()`
- HFI 静音：构造第二参数 `Mute`，重写 `set_current/set_torque`，设置后滞回判断，静音时 `current_ref/torque_ref` 置零
- 目标电流 < `enter_current`（1 A）且转速 < `enter_speed`（10 rpm）进入静音，目标电流 > `exit_current`（2 A）才退出
