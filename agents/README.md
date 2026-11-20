# AI 工作区

本目录供 AI 自己使用，存放工作脚本、测试程序、分析记录和中间产物。通用规则见根目录 `AGENTS.md`

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
├── README.md          本说明
├── TODO.md            跨对话的规划、已确认决定和待办
├── docs/              架构与模块文档，索引见下文
├── out/               统一存放临时输出和中间产物，按任务放入 out/<任务名>/，已被 .gitignore 忽略
└── tasks/
    └── NN-名称/       一类任务一个目录，NN 两位递增序号，名称用 snake_case
```

- 开始任务前先查任务索引，相似任务复用已有目录
- 任务目录可放 `README.md` 记录目标、复现命令、结论和遗留问题
- 日志、固件等临时输出和中间产物统一放在 `agents/out/<任务名>/`，各任务目录禁止创建 `out/`
- Python、Shell、Node 验证脚本将输出根目录设为 `agents/out/<任务名>`；任务目录只保存脚本、测试源代码和记录文档
- 保留可复现的脚本和结论，删除失效临时文件
- `docs/` 是长期项目知识，改代码时同步修正；`tasks/` 是单类任务过程，结论成为长期知识时提炼进 `docs/`
- `docs/` 随对应代码同一提交更新；仅任务记录或索引的变更单独提交，信息以 `agents:` 开头

## 文档索引

- 修改前：`docs/conventions.md`（编码、命名、SWD 类布局）、`docs/workflow.md`（协作、验证、交付）
- 整体分层与入口：`docs/architecture.md`；应用：`docs/application.md`
- MiniPC 导航协议、双 CDC 和坐标转换：`docs/minipc.md`
- `lib/bsp/**`、`boards/**/bsp/**`：`docs/bsp.md`；电机：`docs/motor.md`
- CMake 与工具链：`docs/build.md`；验证与人工审计：`docs/verification.md`
- IMU：`docs/imu.md`；算法原理按需读 `docs/imu-principles.md`
- 单位迁移：`docs/unit-migration.md`；蜂鸣器曲目：`docs/music.md`
- 硬件资料：`reference/电机/` 对应型号、`reference/C板/`、`reference/达妙mc02/`
- 裁判协议和比赛规则：`reference/<年份>规则/`，默认最新年份，先搜索关键词再局部读取

只读取当前任务涉及的文档，相似任务复用下列目录。

## 任务索引

- `01-mc02_bsp/`：mc02 BSP 适配、C 板外设类重构、CherryUSB、`lib/attitude`；复现构建、EL05 私有扩展帧适配与协议回归测试、CAN2 降载、7025/PCAN ID 干扰分析、7025/P630 总线调整与待上板验证项
- `02-buzzer_timbre/`：蜂鸣器复音 PWM-DAC 音色探索与样机归档；正式实现在 `lib/melody`
- `03-quad_steer/`：底盘重构（四舵移植、矩阵底盘、统一功率控制）、PID 回算、UI 下放、机构库掉线处理；已确认决定与上车核对项
- `04-leg_chassis/`：轮腿底盘从 SOLDIER_LEG_4 迁移；IMU 极性核查、电机映射与虚拟 IMU 方案
- `05-dummy_target/`：PR #3 靶车移植到 `app/dummy`；映射、删减与上车核对项；follow PID 微分尖峰只读采样与分析
- `06-super_cap_capacitance/`：超电在线容量/ESR 辨识（RLS）与虚拟缓冲能量功率控制；主机仿真与上车核对项

- `07-can_tx_latest/`：CAN 发送“同 ID 只留最新”方案（已撤销）；撤销延迟、FIFO 空洞等实测硬件知识与上板测试写法
- `08-attitude_cpu/`：attitude 示例 -Og 下 Eigen 不内联导致 CPU 超时；编译变体统计与上板周期测量
- `09-leg_lqr/`：Python 复现 SOLDIER_LEG_4 轮腿 LQR（替代 MATLAB）；重心偏移下位移稳态误差与漂移分析、LQI 对比、首版 Q_s=60 参数实车对照
- `10-global_refactor/`：全局优化（Timer/PID dt、电机默认参数与符号修复、持有与 update 约定、类布局）；外部 worktree 成员展平、angle 命名与多圈零点、成员六组与函数调用序排序、最小权限及验证记录
- `11-shooter_state/`：17 mm 拨弹状态机与 dt 累加计时、电机原生输出接口、云台与发射机构迁移及主机测试
- `12-vt13_control/`：VT13 操控量解析（键盘斜坡、云台、发射、自瞄）、leg_up 与 sentry 迁移、sentry 移除 FSi6X；主机测试与上车核对项
- `13-referee_relay/`：leg_down 裁判串口经 CAN 转发（RefereeRelay）、Referee 改为只收 CAN、VT13 解析图传链路 0x0311；Referee 收发拆分为 Referee、RefereeSender（含 UI、0x0120、0x0308）与 RobotUi；主机测试、ARM 单文件编译与上车核对项
- `14-upstream_merge_review/`：rmpp-czk 分支合入上游的预检与执行（原 12）；保存本地基线、EL05 适配、折叠云台移入 jelly_sentry 与局部编译验证
- `15-minipc_nav/`：rmpp-czk 导航 17 字节协议（原 13）、独立 CDC、底盘随头/小陀螺、坐标转换与 pitch1 全程失能验证
- `16-jelly_sentry_merge/`：jelly_sentry 与依赖库从 rmpp-czk 并入 master；VT13、Referee 收发拆分适配、行为变化与上车核对项
- `17-app_unify/`：sentry 改名 steer_sentry，两台哨兵与 leg 统一 app 结构（setup/loop/send、app 层 referee）、VT13/自主控制拆分、删除 insta360 与 FSi6X、leg_up 接入裁判与 UI；修复项与上车核对项
