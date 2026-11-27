# AI 工作区

本目录按主题存放 AI 的任务记录、脚本和模块知识。通用规则见根 `AGENTS.md`；本文件是入口和主题索引。

## 开始任务

1. 读根 `AGENTS.md`（赛事语境、沟通方式、代码约定）
2. 读本文件的主题索引，确定任务属于哪个主题
3. 读 `TODO.md`，核对未决事项和待上车项
4. 读该主题的 `README.md` 和其中索引的文档、任务记录
5. 改代码前读 `agents/02-conventions/conventions.md`；协作与验证读 `agents/02-conventions/workflow.md`

## 用户需求理解

- 用户维护 RMPP STM32 机器人控制工程，代码需要人工逐行审计后才上车测试
- 优先简洁、直观、统一的实现，反感兼容包装、冗余中间层和为假设需求做的兜底
- 工作区可能有用户和多个 AI 并行修改，只动本任务内容
- 工程目录保持干净：AI 临时文件不进入 `app/`、`lib/`、`boards/`、`tools/`
- 帮用户调通工程时使用默认 `build/`；其他 AI 产出留在本目录
- 回答和文档使用中文，先给结果再说明变化、验证和限制

## 目录结构

```text
agents/
├── README.md          本说明：入口和主题索引
├── TODO.md            跨对话未决事项：待确认决策、待上车核对、硬件问题、后续任务
├── 01-board/          板卡、BSP、构建、IMU、电机
├── 02-conventions/    编码约定、协作流程、验证、架构与 app 组织
├── 03-control/        底盘、靶车、云台与发射、VT13
├── 04-leg/            轮腿底盘、LQR、上下板、力矩与功率分配
├── 05-comm/           裁判转发、协议写法、MiniPC 导航与自瞄
├── 06-sentry/         哨兵应用结构、上游合并、jelly_sentry 并入
├── 07-power/          超级电容
├── 08-music/          蜂鸣器
├── 09-guides/         可复用的 AGENTS.md
└── tmp/               日志、固件、构建产物和临时交付，不入库，随时可能清空
```

- 每个主题的 `README.md` 是该主题的入口，写当前状态、关键约定、待办和文件索引
- 原始任务记录作为主题下的子目录保留，含脚本和测试源码；新任务归入现有主题，确实不属于任何主题时再新建 `NN-主题`
- 脚本输出写到 `agents/tmp/<任务名>/`（Python 用 `ROOT / "agents/tmp/<任务名>"`，Shell 用相对路径），运行时按需创建
- 历史产物 `agents/out/` 继续列入忽略项，其中包含当前 Ember 使用的 OpenOCD 工具和诊断原始数据；新增产物统一写入 `agents/tmp/`
- 长期结论写进主题文档并在主题 README 索引；一次性过程留在任务子目录；办结的 TODO 条目从 `TODO.md` 删除
- 主题文档随对应代码同一提交更新；仅记录或索引的变更单独提交，信息以 `agents:` 开头

## 主题索引

| 主题 | 内容 | 入口 |
| --- | --- | --- |
| `01-board` | C 板/mc02 的 BSP、构建、IMU、电机协议；EL05 与 CAN 实验记录 | `01-board/README.md` |
| `02-conventions` | 编码与类布局、协作与验证、目录与接口、app 组织；全局重构记录 | `02-conventions/README.md` |
| `03-control` | 四舵轮、靶车、拨弹与 GimbalClassic、VT13 操控量 | `03-control/README.md` |
| `04-leg` | 轮腿底盘与上下板、LQR、UI 槽位、力矩与功率分配 | `04-leg/README.md` |
| `05-comm` | 裁判串口转发、协议解析写法、MiniPC 自瞄与导航 | `05-comm/README.md` |
| `06-sentry` | jelly_sentry/steer_sentry 应用结构、上游合并与移植 | `06-sentry/README.md` |
| `07-power` | 超级电容辨识与功率控制 | `07-power/README.md` |
| `08-music` | 蜂鸣器播放与音色实验归档 | `08-music/README.md` |
| `09-guides` | 可复制到其他项目的 AGENTS.md | `09-guides/README.md` |

## 跨项目文档

面向队员和外部协作者的文档在根目录 `docs/`：

- `docs/chassis-leg.md`：轮腿控制思路，面向刚接触代码的队员
- `docs/minipc_protocol.md`：发给上位机的自瞄与导航对接文档

硬件手册和比赛规则在 `reference/`：`reference/电机/`、`reference/C板/`、`reference/达妙mc02/`、`reference/<年份>规则/`。裁判协议和赛规默认用最新年份，先搜索关键词再局部读取。
