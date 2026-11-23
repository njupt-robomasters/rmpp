# 上游合并与接口适配

## 合并预检（2026-11-24，尚未执行合并）

- 本地 `master` 为 `10e2767af510120fd8021bc566367c8bfad8daa7`，已拉取的 `origin/master` 为 `741379a4fb042e9491f3d4ced60e9d6a2dd85ef7`，共同祖先 `7e57d2b396af1b7fe72804dce86a7ba9795dc191`；本地独有10提交、上游独有11提交
- 当前15个已跟踪文件有未提交修改，暂存区为空；未跟踪路径与上游没有同路径覆盖。最近的 yaw 窗口、PID、导航约定等包含在工作区快照中，实施前应先保存这些成果
- 复用 `node agents/tasks/12-upstream_merge_review/review.cjs`，在任务 `out/review.git` 的独立对象库和索引预演；原分支、真实索引和工程源码保持原状。本轮仅预演和静态核对，未编译、测试或操作硬件
- 仅合并已提交HEAD及纳入当前已跟踪修改的两次预演，冲突路径集合相同，均为29项：15项内容冲突、11项双方新增、3项修改/删除。工作区候选树为 `ec14041fed90996fb9edad7da44042799d1d0799`，完整路径见 `out/summary.json` 和 `out/working_merge.txt`

### 冲突分布与推荐处理

| 范围 | 数量 | 处理方向 |
| --- | ---: | --- |
| `app/jelly_sentry/` 的 app、chassis、gimbal、shooter 各一对 hpp/cpp | 8 | 保留本地整车行为及实测配置，逐项迁移上游VT13、裁判和UI接口 |
| `app/sentry/app.cpp`、`chassis.cpp`、`misc.hpp` | 3 | 上游将该应用重组为 `steer_sentry`；对照迁移本地差异后采用新目录和预设 |
| `lib/minipc/aim.cpp`、`nav.cpp`、`nav.hpp` | 3 | 21/18字节自瞄和17字节导航帧已经一致；整合CRC/浮点编解码实现，保留本地导航坐标及flag语义 |
| `lib/motor/el05.hpp` | 1 | 上游把temperature统一移入Motor；移除子类同名字段，保留私有扩展帧、节点和力矩语义 |
| `lib/referee/referee.hpp` | 1 | 采用上游接收/发送拆分和新配置，保留hurt/shoot公开测量及hurt_sequence |
| `lib/robot/shooter/shooter_17mm.*` | 2 | 本地按目标力矩阈值卡滞，上游按PID输出持续饱和卡滞；推荐先保留本地实机策略和参数 |
| `.gitignore`、文档及历史测试 | 11 | 合并忽略规则与知识记录，整理重复迁入的任务记录；按最终接口调整相关脚本 |

### 自动合并后的结构与行为风险

- **同名云台实现重复**：候选树同时存在 `app/jelly_sentry/gimbal_fold.*` 和上游新增 `lib/robot/gimbal/gimbal_fold.*`，两者均定义全局 `GimbalFold`，app/lib的递归源码收集会编译两份，形成重复定义。推荐将当前本地实现迁入上游库路径并只保留一份，窗口接口及所有调用同步迁移
- **当前云台行为**：保留 `is_detect=1` 小轴 ±40° 窗口、越界大轴补足，`is_detect=0` 大轴追随/小轴补偿，底盘转速前馈及二次曲线已删除的状态。上游库实现尚无窗口接口；其末尾还用大轴参考重写头部目标，移植时继续保留本地最终头部目标保持策略
- **实测参数**：保留yaw1速度环kp=3、ki=0.15、位置环kp=5；yaw2速度环kp=2、ki=0.1、位置环kp=15，小轴offset=3.192 rad；保留pitch2当前参数、5 Nm输出上限和IMU 50℃。上游相应配置仍是较早初值
- **导航语义**：本地速度为最终IMU头坐标系，flag=0为60 rpm小陀螺、flag=1随头；上游速度为大yaw雷达系，先按负小yaw角旋转，flag=0随头、flag=1小陀螺且转速随平移变化。推荐沿用当前本地约定、人工接管策略和pitch1固定失能，代码及协议注释一起保留
- **遥控接口**：上游原始量移至 `vt13.rc`，新增 `vx/vy/wr`、`yaw_speed/pitch_speed`、`bullet_speed/bullet_freq`、`is_enable/is_aim`。原地保留旧调用会产生编译错误；直接取上游app会移除FSi6X、鼠标角加速度限制和全景检查分支，并改变photo/trigger、反向拨弹与人工接管行为。推荐迁移接口时逐项保留本地映射，避免重复叠加上游已合成的摇杆/鼠标输入
- **裁判/UI**：上游移除 `referee/ui.hpp`，新增 `RobotUi`、`RefereeSender` 和 app侧 `referee.*`；`Referee::Config.bus` 改为 `can`，发送从Referee分离。候选仍保留本地 `misc.*`，需统一全局实例、头文件、发送调度和逐字段UI刷新；VT13构造时应填入当前实际速度/弹速/弹频配置，上游这些字段默认0
- **卡弹与射击参数**：本地 `block_torque=5.5 Nm`、PID/电机上限6 Nm、reverse_time=0.1 s、heat_protect=100；上游删除block_torque，按PID饱和判定，PID/电机上限5 Nm、reverse_time=0.25 s、heat_protect=80。推荐先保留本地卡弹和操控行为，把判定规则改变单独决策
- **构建与板卡**：CMakePresets自动合并，接纳sentry→steer_sentry预设变化；逐对象核对候选树的mc02 .ioc和7个已修改生成文件，均与本地快照一致，CAN2 ENABLE/FIFO配置保留。上游没有修改这些生成文件
- **忽略与脚本路径**：本地忽略 `agents/tasks/**/out/`、`agents/.tools/`，上游改为 `.agents/`、`agents/out/`；合并时保留既有工具/产物忽略，并同步新脚本输出目录约定。上游还将本地早期12/13任务复制为14/15目录，需整理索引与交叉引用

实施建议：先保存当前成果，以本地整车行为为保留基线接纳上游库/遥控/裁判/UI结构，解决重复类和全局实例后仅编译受影响文件；验证范围保持精简，完整构建和硬件操作按用户后续要求执行

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
- 用独立裸仓库和索引模拟：只合并HEAD有CMakePresets.json及agents/docs/application.md两项冲突；纳入当前已跟踪修改后，新增agents/README.md，共3项文本冲突。原工作区、分支和真实索引均未切入合并
- CMakePresets需保留jelly_sentry及其调试入口，接纳上游leg/leg_down、leg/leg_up路径和leg-leg_down、leg-leg_up名称，删除已被上游移除的infantry预设；两个文档冲突按本地调试成果及上游模块改动整合
- 上游Motor将position改为angle/angle_multi，超时清零单圈angle、保留angle_multi；移除raw_current/raw_torque/raw_speed/raw_position，改用can_callback(const Feedback&)的可选测量。EL05需按新接口传入反馈，保持私有扩展帧、节点18/34、力矩输出与方向不变
- GimbalFold保持jelly_sentry独立逻辑；4轴角度读取、yaw零位校准与gimbal_motor调试需迁移angle接口，并审计掉线角度归零后的恢复行为。imu_calib直接读取的Imu::temperature已被上游改为private，需要适配现有校准流程
- 类布局/最小权限调整同时覆盖SuperCap、Shooter17mm、遥控、裁判等，需更新依赖类内部布局的主机测试与Ember观察路径；本地EL05协议测试的Motor回调同样需迁移。VT13 C/N/S、扳机+左摇杆逻辑与超电控制保留
- 本地block_torque与block_time=0.2、reverse_time=0.25在候选树自动保留；本地CAN2 AutoRetransmission=ENABLE的.ioc与fdcan.c一致且未发生冲突。实机方向与其他参数按当前文件快照保留，包括rub1反向、rub2默认方向、shoot反向
- 新增check_candidate.py：在out内展开候选树，复用当前ARM编译参数做-fsyntax-only检查，不改工程源码。7个文件中EL05因旧raw字段/无参回调失败，gimbal_fold及gimbal.cpp因position接口失败；shooter_17mm.cpp、app.cpp、chassis.cpp、shooter.cpp通过。未运行完整构建或硬件操作
- 结果在out/summary.json、working_merge.txt及candidate_compile.json，候选树16fdccba7c7b9931ea560cb3f2a7cb2446ec34a7。下一步先保存当前成果，再解决3项文本冲突、迁移接口并执行相关ARM对象编译和行为回归；当前结论为可合并、需要适配后验证

2026-11-15：按用户要求先提交当前成果，再合并上游、启用 CAN2 自动重发并迁移接口。以下预检基线保留用于复核，实施结果见末节

## 基线与复现

- 本地 master：`542dcc254abafc052467cb22bb2b1906a2aa10f8`
- 已执行 `git fetch origin`，上游 origin/master：`f3b8e14687308db0b2cbcb8b1057bd2c6c77dbce`
- 共同祖先：`564c678d5e6714a4c35aaf54d4033d68a7d8d378`
- 本地独有 1 个提交，上游独有 26 个提交；暂存区为空
- 原有未提交内容差异为 23 个已跟踪文件；大量 Drivers/CMake 文件虽显示 M，Git 规范化后的 diff 无内容差异，不认定为驱动升级
- 未跟踪文件与上游文件没有同路径覆盖；未纳入临时合并快照
- 运行 `node agents/tasks/12-upstream_merge_review/review.cjs` 复现，先按需 fetch；结果保存在被忽略的 `out/`
- 脚本使用独立裸仓库、索引与对象库，通过 alternates 只读引用工程对象；不移动工程分支或改写工程索引

## 文本冲突

只合并已提交 HEAD 时，只有 `CMakePresets.json` 冲突。将当前已跟踪文件的未提交改动一起纳入后，共 4 个冲突文件：

| 文件 | 内容与处理方向 |
| --- | --- |
| `CMakePresets.json` | 本地插入 jelly_sentry 与调试预设，上游把 leg 拆成 leg_down/leg_up；需要同时保留本地预设和上游拆分结果 |
| `agents/TODO.md` | 两侧新增任务记录，按现状整合 |
| `agents/docs/motor.md` | 本地 EL05 MIT 说明与上游电机接口说明，需随接口迁移同步更新 |
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
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/tasks/12-upstream_merge_review/verify_baseline.py
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/tasks/12-upstream_merge_review/compile.py
```

- 29 项基线/配置检查：PID 数值、发送周期、IMU 安装和俯仰零点/限位、11 个入口的实际发送开关、预设与 CAN2 配置一致性
- 整车 rmpp_app、rmpp_lib、rmpp_bsp 及 FDCAN 初始化对象局部编译通过；10 个 jelly_sentry_debug 入口 rmpp_app 对象全部编译通过，零警告
- clang-format 22.1.0 格式化并检查本次 16 个手写源码文件，`git diff --check` 检查任务改动
- 编译命令与日志保存在 `out/compile_summary.json`、`out/*_compile.log`，工程对象产物仍使用默认 `build/`
- 本轮仅对象编译与静态基线核对；未完整链接、运行控制算法测试或操作硬件。CAN2 重发/FIFO 的实际负载、7025 干扰与 EL05/4310 反馈需用户审计后上板核对

## 合入上游 09766fa（2026-11-18）

- 本地基线 `a220339`。继续工作区已开始的合并，保留两侧 TODO 和上游单份裁判配置，并保留折叠哨兵读取的受击记录、计数及实测弹速
- 上游删除云台基类后，将 `GimbalFold` 移入 `app/jelly_sentry`。四轴控制独立执行，整车和两种云台调试目标共用实现。保留实机调参、CAN 绑定与发送调度
- 迁移裁判在线字段 `is_connect`，移除与已删除示例对应的 `examples-gimbal` 预设
- `python agents/tasks/12-upstream_merge_review/verify_gimbal_fold.py` 对比 `a220339`。本机用 Zig 0.15.2 的 `zig c++`，通过 `--cxx <zig.exe> c++` 指定。6,000 周期、144,000 项比较通过，覆盖模式切换、掉线恢复、限位、重力补偿和四轴电流与力矩输出。主机编译提示第三方 Eigen 和 GCC 专用 pragma 的兼容警告
- `python agents/tasks/06-super_cap_capacitance/verify_jelly_sentry.py` 的 750 项超电策略检查通过
- Arm GCC 局部编译通过，覆盖公共库及 jelly_sentry、三个相关调试目标、sentry、infantry、leg_up、examples-motor、test。两个云台调试目标对象部分链接后无未解析的 GimbalFold 符号。格式与 diff 检查通过，未做完整固件链接或硬件验证
