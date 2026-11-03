# 电机库开发约定

仅在修改 `lib/motor/**` 时读取本文档

## 构造与配置

- 各层构造函数只接收并传递 motor 唯一的 config 参数
- 型号默认值由子类通过 Config 成员补齐
- 不另设全局型号配置包装
- 配置按 `bus/drive/install` 组织
- `drive` 保存 kt、电流与转矩量程、减速比，`install` 保存零点和方向
- 自行发送的协议使用 `tx_id/rx_id` 明确收发方向；DJI 组帧仅配置 `rx_id`

## 基类与子类

- 适配父类重构时优先修改子类
- 父类只保留通用控制和反馈修正
- 最终型号负责注册回调、筛帧和解析型号特有字段
- DJI 型号解析反馈后直接调用 `Motor::can_callback()` 完成通用反馈修正
- 其他协议子类解析后写入 `raw`，再调用 `Motor::can_callback()` 修正反馈并刷新在线状态
- 周期 `update()` 只完成离线检测，闭环由机构库完成

## 协议刻度

- 仅在一处使用的缩放刻度直接写在换算处
- 不增加 `feedback_scale`、`command_scale` 等单用途配置字段

## 零值与限幅

- 零值 `kt` 补为 1
- 零值 `drive.reduction` 补为 1
- `max_current/max_torque` 仅一项为零时，通过 kt 补齐
- 两项全为零时保持零
- `set_current()` 先将非有限值置零，再按 `max_current` 限幅并换算转矩
- `set_torque()` 先将非有限值置零，再按 `max_torque` 限幅并换算电流
- 对应最大值为零时，输出限制为零

## DJI 组帧

- app 创建长度为 8 的零初始化组帧，调用各电机 `write_can_frame()` 填入对应两字节后统一发送
- 型号根据控制帧 ID 和反馈 ID 校验分组、选择协议刻度并计算槽位
- 校验失败返回 `false` 且不修改帧；离线或未使能时成功写入零命令

## GM6020 电压帧

- 电压帧 0x1FF/0x2FF 把 `current_ref` 按 3 A ↔ 25 V 换算为电压，电流帧 0x1FE/0x2FE 直接发电流
- 反电动势前馈在电机定义时用 `BackEmf{gain, ke}` 给出，发送时叠加 `gain·ke·speed` 并限幅 ±25 V；`ke` 默认手册 1 V/13.33 rpm，`gain` 默认 0 不补偿
- 只有一个构造，不补偿时调用方传 `{}`
