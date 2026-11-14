# IMU：配置、解算与审阅

需要理解 EKF 并审阅实现，先读 [IMU EKF：从公式到代码审阅](imu-principles.md)。

IMU 分为两层：BSP `Imu` 负责硬件读取、零偏校准和安装方向修正，可单独使用；`lib/attitude` 是可选的姿态解算。`Attitude` 为抽象父类，调用 `Imu::update()` 并由 q 统一输出欧拉角和累计 yaw；子类按算法命名，只实现 `solve(gyro, accel)` 更新 q，目前有 `QuaternionEkf`。

数学类型统一使用随项目固定的 Eigen 3.4.0。`Vector2f/3f/4f`、`Quaternionf`、`Matrix3f/6f`
均为定长 `float` 类型，定义在 `math/eigen.hpp`；解算不使用动态矩阵、堆分配或 Eigen 向量化。

| 文件 | 职责与审阅重点 |
| --- | --- |
| `lib/bsp/imu.hpp/.cpp` | 接口、配置与两板共用的零偏校准 |
| `boards/*/bsp/imu.cpp` | 板卡实现 `Imu::update()`：BMI088 初始化时序、寄存器事务、原始量解码、恒温加热，再调用共用校准与修正；各板只有 SPI、片选和加热定时器不同 |
| `lib/attitude/attitude.hpp/.cpp` | 抽象父类：读取 Imu、欧拉角和累计 yaw |
| `lib/attitude/quaternion_ekf.hpp/.cpp` | 子类：四元数和 X/Y 剩余零偏组成的六状态 EKF |
| `lib/math/eigen.hpp` | 唯一 Eigen 入口：按 O2 包含 Eigen 模块，定长类型 using 声明 |
| `lib/math/angle.hpp` | `Angle`、`rotate`、`EulerAngles`、`euler_to_quaternion`、`quaternion_to_euler` |

`Imu::update()` 首次调用时初始化传感器，失败后下次重试；读取成功后依次执行一步加热闭环、推进校准、零偏/重力模长/安装方向修正；读取失败关闭加热，输出保持上次值。`update()` 无返回值。初始化保留两次 80 ms 复位等待。`Imu` 输出已修正的机体系 `gyro`（rad/s）、`accel`（m/s²）和 `temperature`（°C）。

`Attitude::update()` 先调用 `Imu::update()`，再调用子类 `solve()` 更新 q，再输出欧拉角和累计 yaw。应用只需填写与默认值不同的项目：

```cpp
inline Imu imu({.orientation = euler_to_quaternion({.yaw = -90 * deg}),
                .gyro_offset = Vector3f(-0.00067f, 0.00137f, 0.00146f),
                .gravity = 9.547f});
inline QuaternionEkf attitude(imu, {}); // 可选；第二参数覆盖 QuaternionEkf::Config
```

Config 仅作为构造输入，固定参数展开到私有 const 成员；gyro_offset、gravity 保留为可校准字段。`Imu` 的温控 PID 借用内部配置，不应复制或移动；`Attitude` 保存 `Imu&`，`Imu` 须在其生命周期内有效。使用姿态解算时只调用 `attitude.update()`，不再单独调用 `imu.update()`。

## 坐标与公开转换

- 右手系，roll 绕 X、pitch 绕 Y、yaw 绕 Z；`R = Rz(yaw) Ry(pitch) Rx(roll)`
- `q = [w, x, y, z]`，表示机体到世界的旋转；静止时加速度参考方向为世界 +Z
- `gyro`、`accel` 为已校准并完成安装修正的机体系量，分别为 rad/s、m/s²；`gyro` 尚未扣除 EKF 内部估计的剩余零偏
- `yaw/pitch/roll` 使用弧度；`yaw_total_angle` 连续累计，不归一化。单次更新的 yaw 变化应小于 π；pitch 接近 ±π/2 时欧拉角本身存在奇点，应使用 q

| 旧接口 | 新接口 |
| --- | --- |
| `eular_angle_to_quaternion` | `euler_to_quaternion` |
| `quaternion_to_eular_angle` | `quaternion_to_euler`，返回 yaw、pitch、roll |
| `body_frame_to_earth_frame` | `q * body`（Eigen 直接运算） |
| `earth_frame_to_body_frame` | `q.conjugate() * world` |

欧拉角转换为 `math/angle.hpp` 自由函数；坐标系变换直接用 Eigen 四元数运算，不另设包装。q 应为单位四元数。

原安装矩阵实际为 `Rz(-old_yaw) Rx(old_pitch) Ry(old_roll)`，不能对任意安装角简单交换字段。当前配置已按相同物理旋转迁移：hero 的旧 roll=180° 改为 pitch=180°，sentry 的旧 yaw=90° 改为 yaw=-90°，其余零安装角不变。

**云台读取 `attitude.pitch`，它表示绕 Y 的角度。** 本次按确认方案只规范 IMU 定义并保留原安装映射，没有为维持旧绕 X 俯仰行为重排机体轴；上车前需核对机械俯仰轴及控制反馈。

## 参数：全部有默认值

硬件由板卡 BSP 固定，不再由 app 绑定。`Imu::Config` 包含安装、标定和温控参数，`QuaternionEkf::Config` 包含 EKF 参数。

| 位置 | 项目数 | 默认值与作用 |
| --- | ---: | --- |
| `Imu::Config::orientation` | 1 | 单位四元数；传感器到机体安装旋转，可用 `euler_to_quaternion()` 从欧拉角构造 |
| `Imu::Config::gyro_offset` | 3 | `Vector3f`，0 rad/s；传感器坐标系静态零偏 |
| `Imu::Config::gravity` | 1 | 9.81 m/s²；静止原始加速度模长，必须大于 0，修正比例为 9.81/gravity |
| `Imu::Config::temperature` | 1 | 45°C |
| `Imu::Config::temperature_pid` | 7 | kp=0.25、ki=0.01、kd=0、ff=0、max_i=0.4、max_out=1、fc=0；输出限制为 [0,1] 加热占空比，BSP 按 `ARR × 占空比` 写 CCR（用 ARR+1 时 C 板 TIM10 满占空比会使 16 位 CCR 溢出为 0、加热停止）；mc02 加热电阻接 12~24 V 电源输入，BSP 再乘 0.05 衰减（参考官方例程 CCR 上限 500/10000），防止烧毁 PCB；C 板不衰减 |
| `QuaternionEkf::Config::quaternion_noise` | 1 | 10；四元数过程噪声强度，非负，每周期乘 dt |
| `QuaternionEkf::Config::bias_noise` | 1 | 0.001；零偏过程噪声强度，非负，每周期乘 dt |
| `QuaternionEkf::Config::accel_noise` | 1 | 1e7；单位重力方向的观测方差，正数，增大后更信任陀螺仪 |
| `QuaternionEkf::Config::bias_fading` | 1 | 1；每周期零偏协方差保留系数，范围 (0,1]，减小后遗忘更快 |
| `QuaternionEkf::Config::innovation_threshold` | 1 | 1e-8；归一化残差门限，正数，增大后容许更大的观测偏差 |

`accel_noise`、`innovation_threshold` 应结合调节，前者改变残差检验的尺度。默认值延续原调参量级，不应把 1e7 当作真实加速度方差；量测已归一化。`bias_fading` 按周期生效，改变循环频率时需重新审视。

温控默认值由 `Imu::Config` 提供，通用 `Pid::Config` 自身的默认值仍是零。若只覆盖温控 PID 的一个字段，先从 `Imu::Config{}.temperature_pid` 复制，再修改该字段；直接写 `.temperature_pid = {.kp = ...}` 会替换整个 PID 配置，使未填写的限幅等字段归零。已有应用没有重复填写整套默认 PID 参数。

内部仍有这些可调整策略，暂不暴露为运行时接口：

| 位置 | 当前值 |
| --- | --- |
| 校准 | 固定 5 s、加速度模长极差 0.5 m/s²、重力偏差 0.5 m/s²、角速度极差 0.15 rad/s、平均零偏上限 0.01 rad/s |
| EKF 初始协方差 | q 对角 100000，零偏对角 100，非对角 0.1，沿用原模型量级 |
| EKF 静稳判断 | 角速度模长 <0.3 rad/s，加速度模长在 9.81±0.5 m/s² |
| EKF 残差策略 | 收敛判据为门限的 0.5 倍，自适应增益从 0.1 倍开始减小；静稳连续拒绝 51 次后重建不确定度 |
| EKF 零偏约束 | 收敛时每周期修正不超过 0.01×dt rad/s；预测前对角协方差上限 10000 |
| BMI088 | 加速度 ±6g、800 Hz、normal 带宽；陀螺仪 ±2000°/s、2000 Hz/230 Hz；轮询，不启用数据就绪中断 |

硬件档位与读取缩放必须配套修改；复位等待和 SPI 空读属于协议要求，不作为调参项。

## 校准与解算行为

`Imu::start_calibration()` 请求开始校准，下一次 `update()` 初始化计时和样本计数后开始采样；调用方负责温稳和静止时机。采样固定 5 s，到时按平均零偏判为成功或失败；采样中一旦非静止立即判为失败，不重新开始。校准状态和样本数为内部状态，不对外开放。

成功时一次提交三个零偏与重力模长，用 SWD 观察 `imu.gyro_offset` 与 `imu.gravity` 读取；不会自动写 Flash。失败保留旧标定。采样过程中温控与旧标定解算继续运行。校准归 `Imu`，提交时不再自动清除 EKF 剩余零偏；需要立即重建时由调用方在校准完成后调用 `QuaternionEkf::reset_bias()`（保留 q 和累计 yaw），否则由 EKF 逐渐收敛。BMI088 读取失败时关闭加热，本周期不推进校准和姿态解算。

解算从首个有效加速度建立倾角，初始 yaw=0。它只观测重力方向，没有磁力计或绝对航向源，Z 零偏仍无法估计，yaw 仍会漂移。

EKF 的预测、观测、残差门控、增益、修正与 Joseph 协方差更新顺序展开。使用固定尺寸 Eigen 类型，3×3 `LLT` 求解代替显式求逆，预测和后验四元数均归一化，协方差每次对称化。无效 gyro/dt 不推进，零加速度仅预测；量测求解失败保留有效预测，不清空姿态。持续静稳失配时重建协方差及剩余零偏，避免旧交叉协方差把倾角误差积成虚假的零偏。

已删除通用 Kalman 的动态内存、动态维数、控制输入、量测重映射、用户回调、SkipEq 开关、调试副本，以及未使用的 OLS/ramp 工具、BMI088 全局数据、绑定中间层和未使用寄存器宏。IMU 不再依赖 CMSIS-DSP，未改写 CubeMX 生成的 DSP 链接声明。

## 软件验证记录（2026-10-01）

以下记录在 IMU 拆分为 BSP `Imu` 与 `Attitude` 之前完成，文中的文件与测试脚本路径为当时状态；算法源码未变，只是删除了默认禁用的加速度低通 `accel_filter_fc`（与默认行为一致）

运行主机测试需要 Python、NumPy 和支持 C++23 的 MinGW g++：

```powershell
python tools/check_imu.py --cxx D:/mingw64/bin/g++.exe
```

测试桥接调用实际 IMU、驱动和 EKF 源码，仅替换 SPI/PWM/Timer；矩阵与轨迹参考使用 NumPy 双精度。覆盖固定矩阵乘法、Cholesky 回代、病态协方差、无效输入、转换往返与原地变换、倾斜/倒置启动、单轴/组合运动、不同与抖动 dt、加速度冲击、零偏、跨 ±π、初始化重试、多实例、校准重开/超时/成功提交。

| 指标 | 结果 |
| --- | ---: |
| 1000 组转换最大姿态误差 | 0.0000121° |
| 16 组恒角速度解析轨迹最大姿态误差 | 0.00227° |
| 四元数最大模长误差 | 1.10e-7 |
| 抖动周期含噪组合运动 RMS 姿态误差 | 1.026°，含未估计的 Z 零偏 |
| 注入 X/Y 零偏 | 0.006 / -0.004 rad/s |
| 60 秒后 X/Y 零偏估计 | 0.005917 / -0.003944 rad/s |

结果保存到 `build/imu-tests/results.json`。本次旧代码与固件备份在 `build/imu-before`，不改变 Git 暂存区。额外对照编译了备份中的原 QuaternionEKF/Kalman C 源码，用主机 float 矩阵适配替代 ARM 二进制，并关闭旧代码的严格别名优化；因此它是算法回归对照，不是板端指令级等价验证。

同一随机种子、1 ms 周期，陀螺仪噪声标准差 0.001 rad/s、加速度噪声 0.03 m/s²、固定零偏 [0.004,-0.003,0.002] rad/s：

| 30 秒含噪轨迹 | 旧版 RMS | 新版 RMS |
| --- | ---: | ---: |
| 静止 | 2.049° | 2.044° |
| 匀速偏航 | 1.874° | 1.890° |
| 组合转动 | 1.645° | 1.643° |

普通轨迹误差接近，未声称全面优于原调参。旧版四元数模长误差约 0.00169，新版约 1e-7。无角速度提示的 0.8 rad 倾角突变用于验证异常恢复：新版约 1.406 秒后持续进入 1° 以内，旧版在剩余 58 秒测试期间仍未持续进入；这不是正常物理转动精度测试。对照结果在 `build/imu-tests/comparison.json`。

IMU 示例 Debug/Release 均清理后完整构建成功，分别为 0 错误/0 警告、0 错误/1 警告；剩余警告来自原 `lib/ui/lib/ui_interface.c:202` 数组边界。IMU 自身以 ARM `-Wall -Wextra -Werror` 编译通过。

| IMU 示例 | RAM | Flash | 产物 |
| --- | ---: | ---: | --- |
| Debug | 25448 B | 50388 B | `build/examples-imu-debug/rmpp.elf`、`rmpp.map` |
| Release | 25456 B | 49916 B | `build/examples-imu-release/rmpp.elf`、`rmpp.map` |

ARM `-O2 -fstack-usage` 报告 `QuaternionEkf::update()` 自身栈帧 1408 B；这不包含被调用函数、中断嵌套和外层应用。链接脚本最低栈预留为 12288 B。EKF 对象未引用 malloc/new，单周期 Eigen 矩阵使用栈，尚未测量板端栈高水位或周期耗时。

全 24 个预设已尝试，22 个完整目标构建通过，结果见 `build/imu-verification/results.json` 与同目录日志。哨兵的两个预设各有 4 条编译错误，均因 `app/sentry/gimbal.hpp` 引用已删除的 `motor/lk/mf9025.hpp`；这是当前并行电机改动范围，本次未修改或恢复该文件。除 IMU 示例清理重建外，其余预设为增量构建；RC Release 另记录同一 UI 编译警告和一条 Ninja 日志恢复提示，最终构建成功。

尚未烧录或上车。SPI 当前接口没有传输状态返回，驱动的 ID/寄存器检查不能覆盖所有总线错误；模拟测试也不能替代实测时序、温控响应和机械轴向核对。软件通过后由人工审计决定板上测试。
