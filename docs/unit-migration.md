# 数值单位与既有验证

用途：保留多板卡重构之前的单位约定、验证结论及尚待处理的问题。下述历史测试不是本次重构重新执行的测试；本次结果见 verification.md。

## 单位约定

业务数值采用 float，内部使用 SI 单位；温度保持摄氏度，拨弹系数为发数/弧度。
math::Angle 内部为一个 float，赋值归一化到 [-pi, pi)，支持隐式 float 转换及复合赋值。
普通算术结果为 float；连续多圈 IMU yaw、普通角度误差保持 float。
归一化使用有界二进制余数，支持 constexpr，传递 NaN/无穷值。

| 数据路径 | 内部及边界约定 |
| --- | --- |
| Timer、PID | 秒、Hz；PID 增益对应 SI 误差；负载为比例 |
| M2006/M3508/GM6020/RM6623 | 编码器 /8192 × rev；电流与转矩保持原缩放、截断及反向逻辑 |
| MF9025/CustomMotor | 编码器或角度转换为 rad，转速 rad/s，电流 A；沿用原报文字节序 |
| DM4310/DM6006 | MIT 角度、速度、转矩范围，沿用 12/16 位编码与使能报文 |
| CyberDog | 电流 A，转矩为 current × kt；保持原电流编码 |
| VESC | eRPM 按极对数换算 rad/s；电流 /10、位置 /50 × deg、温度 /10 |
| 底盘与功率 | m/s、rad/s、N、N·m、W、J；保留限幅和方程顺序 |
| 云台与发射 | rad、rad/s；弹速 m/s，发射频率 发/s，拨弹系数 发/rad |
| IMU | EKF 角度从度转换 rad；陀螺 rad/s，加速度 m/s²；总 yaw 不归一化 |
| 加热 | 45 ℃，kp=1，ki=0.01，积分上限 0.4，占空比上限 1 |
| MAVLink | 生成协议、ID、CRC 保持；协议中的度/rpm 在边界转换 |
| UI | 显示前 /deg、/rpm、/pct，随后整数截断；英雄弹速 ×10 只在显示边界 |
| FlashDB | yaw_offset 保存归一化弧度；历史角度制数据需重新校准 |

默认数值保持零初始化，保留原非单位比例因子；未调整 PID。
历史配置审计在本机 build/unit-migration/host/configuration-audit.csv。
例如英雄 yaw 速度 kp 为 1.4323945，pitch 速度 kp 为 2.86478901、ki 为 57.2957802，拨弹系数为 0.95492959 发/rad。

## 历史软件验证

单位迁移基线为 f72e9041bed01fb8e6ded80665dc3a7b63795b9e。
当时保护了用户已有删除和 IDE 修改；历史归档位于忽略的 build/unit-migration，干净克隆不包含它。

- MinGW GCC 15.2、C++17：边界、constexpr 及 200,000 组随机 float 对照 fmod，覆盖 NaN、无穷、最大浮点数及换算因子。
- Arm GCC 15.3.1：同一数学测试单元编译通过。
- 旧/新真实 PID、计时、电机、底盘、云台、发射、IMU 适配、UI、MAVLink 以模拟硬件执行。
- 7,004 组差分观察通过；浮点容差为 1e-6 + 1e-5 × abs(基线)，CAN 字节、MAVLink 含 CRC、UI 整数字段精确比较。
- 额外 2,077 组 UI 截断与 VESC eRPM 量化边界无整数差异。
- 当时补充编译通过 12/13 个调试例程和工程底盘；其余既有失败被保留记录。

| 应用 | Debug RAM / Flash（B） | Release RAM / Flash（B） |
| --- | ---: | ---: |
| hero | 33064 / 108992 | 32992 / 117088 |
| infantry | 32416 / 107644 | 32344 / 114068 |
| sentry | 32880 / 109968 | 32808 / 120448 |

CCMRAM 均为零。单位迁移前 Debug RAM/Flash 分别为 hero 57328/153380、infantry 54016/150652、sentry 56552/158832；hero Release 为 57264/142696。
历史日志、ELF、map 和配置记录在 build/unit-migration/final。

## 保留问题

- engineer 的 Mavlink 构造缺少配置；该应用仍未注册构建。旧 gimbal_motor 调试入口曾引用未定义 motor，本次已由独立示例替代。
- 双舵轮功率限制公式除以 2*a，零电流时可能产生 NaN；旧/新都产生 NaN 不代表公式正确。
- VESC 占空比沿用 /1000 × pct，应结合实际固件协议单独复核。
- 减速电机累积编码器增量后的 Angle 仍归一化，不是无限多圈角度。
- 17 mm/42 mm 射击 UI 阈值方向及 NaN 着色行为仍有差别。
- QuaternionEKF 的类型重解释和 ui_interface 的数组边界警告在历史 Release 中已存在。
- 原单位标签中部分电流、转矩、位置、速度含义不一致；迁移保留其原数值并修正边界单位表达。

历史软件验证没有覆盖真实噪声、中断时序和机械负载。
