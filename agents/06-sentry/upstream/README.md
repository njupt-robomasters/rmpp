# 上游合并与接口适配

状态：审阅与移植已在主工作区完成，待用户审计与上车

## Joushua233/master 合入审阅（2026-11-24 至 11-25，外部工作区）

审阅 worktree 从本地 `45c743f` 建立；对方为 `0bdfd7b`，共同祖先为 `7e57d2b`。直接三方合并会报 30 个路径冲突。本地 `3d71eb8` 已按模块移植过对方 `65b91de` 和当时未提交的导航改动（见 `agents/06-sentry/jelly_merge/README.md`），因此按模块核对上次移植后的新行为，避免把旧接口和调试目录再次带回。

| 项目 | 两边的实际差异 | 本轮处理 |
| --- | --- | --- |
| mc02 探针与掉线记录 | 对方用 HID 恢复了 Horco 探针通信，并记录了两路电机同时掉线的观测 | 配置和证据收入 `agents/01-board/mc02_bsp/`；不更改本机调试设置 |
| 测试工具 | 对方在 `agents/.tools/` 安装主机工具 | 合入该目录的 Git 忽略规则，任务产物仍放本地约定的 `agents/tmp/` |
| CubeMX 生成文件 | `.ioc` 仅版本不同；对应生成代码是空行和 USER CODE 标签名称差异 | 保留本地生成文件 |
| 路径、驱动、预设 | `app/sentry` 在本地已改名为 `app/steer_sentry`；EL05 实现文件两端内容相同；本地已撤掉整车调试预设 | 保留本地结构，不恢复旧入口 |
| 导航报文与模式 | 外部工作区基线为 17 字节、flag 0 随头；本地主线 `5510c9c` 为 24 字节、过洞标志 1 随头。对方用 17 字节、flag 1 随头 | 合入时按主线 24 字节协议翻译：过洞标志 1 对应随头，测试用例与文档已对齐 |
| 导航速度的前方 | 主工作区对接文档以雷达朝向为前方，对方以云台头部为前方 | 用户确认"雷达朝前、云台朝左时往左走"。折叠哨兵直接采用头部系速度，导航和遥控逐轴相加；舵轮哨兵沿用雷达系 |
| 自主行驶时的人工输入与转速 | 本地自主平移只取导航；对方允许人工接管，自主旋转固定 60 rpm | 用户确认导航与 VT13 平移逐轴相加后限幅，小陀螺按合成平移指令线性减速。原地 120 rpm，2.5 m/s 时 90 rpm，5 m/s 时 60 rpm |
| 导航掉线，遥控和方向反馈正常 | 本地自主模式下整车底盘停车；对方撤销导航指令，仍响应摇杆 | 用户确认摇杆推右时车往右走。撤掉自动平移和旋转，保留 VT13 平移；收到新导航帧时恢复叠加，任一方向反馈丢失仍停车 |
| 姿态传感器加热 | 原折叠哨兵把目标温度设为 0 ℃，对方设为 50 ℃；本地 leg 上下板均使用 `Imu::Config` 的默认温控 | 参考 leg，折叠哨兵去掉目标温度覆盖，共用默认的 45 ℃ 和温控 PID |
| 机构参数和控制 | 对方还有云台识别和发射机构改动，其配置针对实测样机 | 保留本地实现 |

24 字节协议与坐标方向：`steer_sentry` 上位机按雷达系发速度，`jelly_sentry` 上位机按云台头部系发速度，两者均由上位机按各自约定填写 `vx`/`vy`；`docs/minipc_protocol.md` 已分别说明。

11 月 25 日验证：`gimbal.hpp` 通过格式检查。`chassis.cpp` 通过 ARM GCC `-Werror` 对象编译。145 项底盘主机测试覆盖断线时摇杆右移、松杆停车、方向反馈丢失停车、重连后速度重新叠加，以及方向与转速约定，均通过。实际升温曲线尚未上板测量。

## 合入上游7e57d2b（2026-11-20）

- 先提交当前工程成果、测试及记录为6815d85，再合入origin/master@7e57d2b；本机工具配置与out临时产物保持未跟踪
- 解决CMakePresets、任务索引和application文档3项冲突；用tools/update_presets.py生成24个有效应用预设，保留11个jelly_sentry入口，接纳leg/leg_down、leg/leg_up迁移及infantry删除，两侧模块文档与任务索引整合
- EL05按新Motor::Feedback传入torque/speed/angle，由基类完成电流换算、方向修正及在线检测；私有扩展帧收发、节点18/34和力矩输出保持原样
- GimbalFold继续为jelly_sentry独立类；角度读取和yaw零位校准迁移为angle。上游单圈angle离线归零，折叠云台在电机在线时刷新既有measure，离线保留最后测量，维持合并前的合成角与控制行为
- 两路云台调试同步angle命名；imu_calib需要温度稳定判定，Imu.temperature维持公开只读，保留其余上游最小权限布局，并修正校准观察量名称
- 已逐文件比对6815d85，shooter.hpp/shooter.cpp/chassis.cpp/app.hpp及CAN2的.ioc/fdcan.c与保存提交一致；云台参数与零点保持原值，实机方向按rub1=true、rub2默认false、shoot=true保存。VT13三挡、扳机+左摇杆、5 m/s传送带目标、5 N·m阈值/0.2 s检测/0.25 s反转及CAN2自动重发ENABLE均保留
- ARM对象编译通过：整车rmpp_app/rmpp_lib/rmpp_bsp、10个jelly_sentry_debug的rmpp_app，以及leg-leg_up、leg-leg_down、sentry、dummy、examples-motor应用；未出现编译警告。未执行完整固件链接、烧录或硬件操作
- 回归通过：独立GimbalFold对比旧实现6000周期/168000项比较（增加四轴掉线保持断言，历史基线仅迁移字段名及旧Motor离线反馈约定）；EL05私有CAN协议；机构原生输出/目标力矩阈值/反转计时；超电750项模式检查。主机Zig编译有Eigen弃用与GCC pragma兼容警告，状态机副本略过Clang不支持的DM浮点模板实例，DM在ARM编译中验证
- 本次手写适配代码与测试完成格式和diff空白检查；完整日志/测试产物放在out，未入库。后续硬件核对新Motor的观察路径、EL05反馈连续性及启用CAN2自动重发后的负载

## 最新合并预检（2026-11-20，执行前基线）

- 已fetch origin；本地HEAD为c8a791fd67bbc44489e0b29c6cb2c881094df199，上游master为7e57d2b396af1b7fe72804dce86a7ba9795dc191，共同祖先09766fa。本地独有5提交、上游独有20提交
- 当前有20个已跟踪文件未提交；新增EL05协议测试需随成果保存，.agents/、.clangd、.settings/及out产物保留在本机。未跟踪文件与上游没有同路径覆盖
- 用独立裸仓库和索引模拟：只合并HEAD有CMakePresets.json及agents/02-conventions/application.md两项冲突；纳入当前已跟踪修改后，新增agents/README.md，共3项文本冲突。原工作区、分支和真实索引均未切入合并
- CMakePresets需保留jelly_sentry及其调试入口，接纳上游leg/leg_down、leg/leg_up路径和leg-leg_down、leg-leg_up名称，删除已被上游移除的infantry预设；两个文档冲突按本地调试成果及上游模块改动整合
- 上游Motor将position改为angle/angle_multi，超时清零单圈angle、保留angle_multi；移除raw_current/raw_torque/raw_speed/raw_position，改用can_callback(const Feedback&)的可选测量。EL05需按新接口传入反馈，保持私有扩展帧、节点18/34、力矩输出与方向不变
- GimbalFold保持jelly_sentry独立逻辑；4轴角度读取、yaw零位校准与gimbal_motor调试需迁移angle接口，并审计掉线角度归零后的恢复行为。imu_calib直接读取的Imu::temperature已被上游改为private，需要适配现有校准流程
- 类布局/最小权限调整同时覆盖SuperCap、Shooter17mm、遥控、裁判等，需更新依赖类内部布局的主机测试与Ember观察路径；本地EL05协议测试的Motor回调同样需迁移。VT13 C/N/S、扳机+左摇杆逻辑与超电控制保留
- 本地block_torque与block_time=0.2、reverse_time=0.25在候选树自动保留；本地CAN2 AutoRetransmission=ENABLE的.ioc与fdcan.c一致且未发生冲突。实机方向与其他参数按当前文件快照保留，包括rub1反向、rub2默认方向、shoot反向
- 新增check_candidate.py：在out内展开候选树，复用当前ARM编译参数做-fsyntax-only检查，不改工程源码。7个文件中EL05因旧raw字段/无参回调失败，gimbal_fold及gimbal.cpp因position接口失败；shooter_17mm.cpp、app.cpp、chassis.cpp、shooter.cpp通过。未运行完整构建或硬件操作
- 结果在agents/tmp/upstream/summary.json、working_merge.txt及candidate_compile.json，候选树16fdccba7c7b9931ea560cb3f2a7cb2446ec34a7。下一步先保存当前成果，再解决3项文本冲突、迁移接口并执行相关ARM对象编译和行为回归；当前结论为可合并、需要适配后验证

2026-11-15：按用户要求先提交当前成果，再合并上游、启用 CAN2 自动重发并迁移接口。以下预检基线保留用于复核，实施结果见末节

## 基线与复现

- 本地 master：`542dcc254abafc052467cb22bb2b1906a2aa10f8`
- 已执行 `git fetch origin`，上游 origin/master：`f3b8e14687308db0b2cbcb8b1057bd2c6c77dbce`
- 共同祖先：`564c678d5e6714a4c35aaf54d4033d68a7d8d378`
- 本地独有 1 个提交，上游独有 26 个提交；暂存区为空
- 原有未提交内容差异为 23 个已跟踪文件；大量 Drivers/CMake 文件虽显示 M，Git 规范化后的 diff 无内容差异，不认定为驱动升级
- 未跟踪文件与上游文件没有同路径覆盖；未纳入临时合并快照
- 运行 `node agents/06-sentry/upstream/review.cjs` 复现，先按需 fetch；结果保存在被忽略的 `agents/tmp/upstream/`
- 脚本使用独立裸仓库、索引与对象库，通过 alternates 只读引用工程对象；不移动工程分支或改写工程索引

## 文本冲突

只合并已提交 HEAD 时，只有 `CMakePresets.json` 冲突。将当前已跟踪文件的未提交改动一起纳入后，共 4 个冲突文件：

| 文件 | 内容与处理方向 |
| --- | --- |
| `CMakePresets.json` | 本地插入 jelly_sentry 与调试预设，上游把 leg 拆成 leg_down/leg_up；需要同时保留本地预设和上游拆分结果 |
| `agents/TODO.md` | 两侧新增任务记录，按现状整合 |
| `agents/01-board/motor.md` | 本地 EL05 MIT 说明与上游电机接口说明，需随接口迁移同步更新 |
| `boards/mc02/rmpp.ioc` | 本地 CAN2 AutoRetransmission=DISABLE，上游 ENABLE 并增加 FIFO 模式；由用户决定 CAN2 参数并操作 CubeMX |

`boards/mc02/Core/Src/fdcan.c` 可自动合并，但须核对其实际初始化与最终 .ioc 一致。其本地改动仅末尾空行，上游修改了 CAN 初始化行为

## 自动合并不能解决的接口不兼容

以下根据实际本地调用与上游声明逐项对照确认，尚未运行编译：

- `EL05`：本地使用 `Motor(config)`、`config.bus/drive/install`、`raw.*`、`is_enable_`；上游改为带型号默认参数的构造、平铺 Config、`raw_current/raw_torque/raw_speed/raw_position`、`is_enable`
- 上游 Shooter17mm 经 `Motor::set_output()` 输出。EL05 应像达妙驱动一样覆盖为力矩输出；否则继承的默认电流语义会改变原 PID 输出单位
- `GimbalFold`：本地依赖旧 Gimbal 构造、`PidOutput`、带尾随下划线的基类状态、旧虚函数签名及零点存取；上游 Gimbal 已移除相关接口，`backward(float dt)`、`update(imu_yaw, imu_pitch)` 需要迁移
- 本地 PID 调用没有 dt，上游 `calculate(dt, err, derr)` 要求持有方提供周期；折叠云台和单轴/拨弹调试入口均受影响
- `app/jelly_sentry` 和 `app/jelly_sentry_debug` 的电机初始化需迁移；上游移除了型号 `KT` 常量、重组底盘配置，不能只修云台和 EL05
- 本地实测安装方向、PID 参数、CAN ID、可写发送周期和临时注释的发送代码均是既有调试成果，迁移时逐项保留并单独审计

## 实施结果

- 保存提交：`4d59574`，包含既有工程改动、生成文件和任务脚本。本机未跟踪 `.agents/`、`.clangd`、`.settings/` 保留在工作区，不加入工程提交
- 合并上游 `f3b8e14`。提交预检记录后 `agents/README.md` 也发生任务索引冲突，实际共 5 个文件；两侧任务与索引均保留
- CMakePresets 同时保留 jelly_sentry 与 10 个调试入口、上游 leg_down/leg_up；移除已由上游淘汰的 leg 预设
- 用户明确指定采用上游 CAN2 配置，冲突段取上游 ENABLE 和 FIFO；保留本地 CubeMX 版本等其他改动。实际 `fdcan.c` 自动合并得到 ENABLE/FIFO，与 .ioc 一致，未调用 CubeMX 重新生成
- EL05 采用平铺配置、原始反馈标量与原生力矩输出，保留 MIT 帧、周期使能与清错逻辑；app 显式配置 5 Nm 和 `5/0.94` A，电流/力矩比例保持原值
- GimbalFold 通过 `backward(dt)` 统一传入 PID 周期；IMU 更新、yaw 零点与持久化移至 app，移除旧输出类型和旧基类接口依赖；保留独立轴 PID、重力补偿与电机限幅跟踪
- 底盘改由 app 更新超电并设置 power_chassis_ref；VESC 显式保留原参数 20 A、20 Nm、reduction=7
- 裁判数据中已有 hurt_data、hurt_sequence、shoot_data 恢复公开，继续支持本地受击方向 UI 与实测弹速，其他字段权限保持上游状态
- 实机 PID、安装方向、零点、CAN ID、发送周期及被注释的发送代码保留；删除的接口职责已迁至 app 或电机原生输出

## 验证与复现

工具：Arm GCC 14.3.1、CMake 4.4、Ninja 1.13；Python 3.12 路径为 `C:/Users/joushua/.local/bin/python3.12.exe`

```powershell
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/06-sentry/upstream/verify_baseline.py
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/06-sentry/upstream/compile.py
```

- 29 项基线/配置检查：PID 数值、发送周期、IMU 安装和俯仰零点/限位、11 个入口的实际发送开关、预设与 CAN2 配置一致性
- 整车 rmpp_app、rmpp_lib、rmpp_bsp 及 FDCAN 初始化对象局部编译通过；10 个 jelly_sentry_debug 入口 rmpp_app 对象全部编译通过，零警告
- clang-format 22.1.0 格式化并检查本次 16 个手写源码文件，`git diff --check` 检查任务改动
- 编译命令与日志保存在 `agents/tmp/upstream/compile_summary.json`、`agents/tmp/upstream/*_compile.log`，工程对象产物仍使用默认 `build/`
- 本轮仅对象编译与静态基线核对；未完整链接、运行控制算法测试或操作硬件。CAN2 重发/FIFO 的实际负载、7025 干扰与 EL05/4310 反馈需用户审计后上板核对

## 合入上游 09766fa（2026-11-18）

- 本地基线 `a220339`。继续工作区已开始的合并，保留两侧 TODO 和上游单份裁判配置，并保留折叠哨兵读取的受击记录、计数及实测弹速
- 上游删除云台基类后，将 `GimbalFold` 移入 `app/jelly_sentry`。四轴控制独立执行，整车和两种云台调试目标共用实现。保留实机调参、CAN 绑定与发送调度
- 迁移裁判在线字段 `is_connect`，移除与已删除示例对应的 `examples-gimbal` 预设
- `python agents/06-sentry/upstream/verify_gimbal_fold.py` 对比 `a220339`。本机用 Zig 0.15.2 的 `zig c++`，通过 `--cxx <zig.exe> c++` 指定。6,000 周期、144,000 项比较通过，覆盖模式切换、掉线恢复、限位、重力补偿和四轴电流与力矩输出。主机编译提示第三方 Eigen 和 GCC 专用 pragma 的兼容警告
- `python agents/07-power/super_cap/verify_jelly_sentry.py` 的 750 项超电策略检查通过
- Arm GCC 局部编译通过，覆盖公共库及 jelly_sentry、三个相关调试目标、sentry、infantry、leg_up、examples-motor、test。两个云台调试目标对象部分链接后无未解析的 GimbalFold 符号。格式与 diff 检查通过，未做完整固件链接或硬件验证

- 2026-11-20 按用户要求将 `GimbalFold` 从 `app/jelly_sentry` 移至 `lib/robot/gimbal/gimbal_fold.*`，代码未改，jelly_sentry 改为包含 `robot/gimbal/gimbal_fold.hpp`；本目录与 15 的测试改用新路径
