# 编码与类布局约定

修改手写代码前读取，模块规则见 `agents/README.md` 主题索引。

## 设计

### 原则

- 简洁、直观、可人工审计优先
- 应用代码及业务模块禁止使用 `static_assert`。字节序、浮点表示和协议帧布局等检查放在测试中
- 同类问题使用统一写法
- 删除重复实现、无效中间层、旧接口和依赖
- 不增加兼容包装，不为假设需求保留兜底
- 逐行确认代码、配置和依赖的当前用途
- `lib/`（含 `lib/third_party/`）不依赖 STM32 HAL、CMSIS 或板卡头文件；硬件相关代码放在 `boards/` 或根目录 `third_party/`

### 流程与复用

- 主要流程在对应接口内完整展开，用空行和简短注释分段
- 只有多处复用的逻辑才拆成辅助函数；类专用的作为成员，多类共用的放在已有公共入口
- 不为少量内容另拆文件

### 接口

- 调用参数保持直接，不为少量指针、长度另建包装结构
- 可选参数和重载只按实际调用需要提供
- 构造参数先设备，最后是 `Config`；设备指针、引用不放进 `Config`
- 配置参数不提供默认实参或省略配置的委托构造，使用默认配置时调用方显式传 `{}`；可选前馈系数等单个标量可默认 0 表示不启用
- 类接口统一为 `set_*`、`update`、`send`：每周期都要刷新的外部测量（如姿态）经 `update(...)` 参数传入，其余输入用 `set_*`
- 明确单位、范围、数据有效期
- 区分实例默认配置与单次覆盖
- 名称表达动作和副作用，区分读取、设置、重新开始
- 设置接口只完成对应设置及必要换算、限幅，不隐含无关清理或重启；业务幂等，不做“与当前值相同则返回”的判断

### 数据与状态

- 配置、引用、运行状态分开；所有权和生命周期明确
- 按值持有的组件（如底盘的 PID）由持有者透传设置，app 尽量只操作最顶层对象
- 硬件设备由 app 定义；只服务一个机构的设备（电机）由该机构持有并调用其 `update()`；被多处使用或与机构业务无关的设备（姿态、超电）由 app 调用 `update()` 和设置，再把结果交给机构
- 公开输出按物理量集中组织
- 成员默认 private，变量不加尾随下划线；逐项核对当前调用，类内使用或仅供 SWD 观察的变量与常量保持 private，外部确需访问的才用 public，子类确需访问的用 protected；公开成员变量外部只读，只经 `set_*`、`update` 修改，不提供读取函数
- 固定配置按值保存为 `const Config config`，使用处直接读取，不再逐项复制同名成员；需要解析默认值、校准或运行时设置的参数按实际用途持有，经 set_* 修改；Motor 使用 `Config config`，零值在构造时补齐，offset 由外部直接维护；VT13 使用公开的 `Config config`，app 可直接修改操控参数；Pid 保留指向调用方可调参数的只读引用
- 积分、微分步长由持有方在 `update()` 开头计一次周期、按本类 `DT_MIN`/`DT_MAX` 限幅，再传给 PID 等组件；`Timer` 不限幅
- 标量成员默认值写 `= 0`，不写 `{}`
- 固定硬件绑定不提供运行时修改
- 成员只保存配置、必要引用和跨周期状态，单周期中间结果使用局部变量
- 不重复保存可由上下文或配置确定的数据
- 不增加无实际用途的初始化、标志或特殊成员函数

## 命名

### 基本格式

- 类型：PascalCase
- 函数、变量、文件：snake_case
- 所有成员变量均不加尾随下划线
- 常量、枚举：UPPER_SNAKE_CASE
- 单位符号、型号：保留领域惯例
- 项目手写代码不声明 C++ 命名空间
- 类专用辅助函数作为类 private `static` 成员；与类无关的文件私有函数和变量用全局作用域 `static`
- 类的跨周期状态作为成员，app 的通常放在文件作用域 `static`，保证类可多实例；`app_main()` 的 `app_timer`、`app_monitor` 在 `while (true)` 内对应使用处定义为局部 `static`
- 仅在成员被同名参数或局部变量遮蔽时使用 `this->`

### 语义

- 名称简短、具体，同一上下文不重复所属对象
- HAL 句柄使用 `hcan`、`huart`、`hspi`、`htim` 等领域惯例
- 避免含混的 `handle` 或近似的 `State/Stats`
- 布尔状态和判断优先使用 `is_*`、`has_*`、`can_*`、`should_*`
- 配置开关、协议字段和 `last_*` 历史状态不机械添加 `is_`
- 计时器命名 `<用途>_timer`：取重新开始计时的位置（`update_timer`、`callback_timer`）或所计对象（`block_timer_`、`note_timer_`），不用裸 `timer_` 或 `timer_xxx` 前缀

## 代码结构

### 电机与闭环

- `Motor` 只负责反馈、在线状态和电流/力矩设置，状态字段直接使用 `current/current_ref`、`torque/torque_ref`、`speed`、`angle`、`angle_multi`
- `Motor::update()` 只维护连接状态，不执行闭环控制；速度和角度 PID 由底盘、云台、发射机构持有
- PID 实现在 `lib/pid/pid.hpp` 与 `lib/pid/pid.cpp`，机构配置直接传入 PID 参数指针，不提供隐式默认 PID 参数
- 电机型号在具体类中保留 `MAX_CURRENT`、`MAX_TORQUE`、`REDUCTION` 常量，构造时作为默认值传给 `Motor`；不使用 kt，电流与力矩按二者之比换算
- PID 由持有方传入 `dt`；不使用的分支调用 `clear()`，clear 后首帧不差分

### 协议接收

适用于 `lib/minipc`（Nav、Aim）、`lib/rc/vt13`、`lib/referee/referee` 等字节流协议；FSi6X 的 SBUS 无校验，按空闲中断整帧接收

- 成员：`raw`（多方向时 `raw_rx`、`raw_tx`）保存最近一帧有效原始帧，断联后保留供 SWD 查看；`rx_fifo` 保存未解析字节，只在接收回调内读写；`callback_timer` 用于断联检测
- `callback(const uint8_t* data, size_t size)` 逐字节 `rx_fifo.push()`，在 `while` 中依次：帧头不符丢弃 1 字节；不足整帧 `break`；校验不符丢弃 1 字节；有效帧整帧出队，复制到 `raw` 后解析，最后置 `is_connect` 并重启 `callback_timer`
- 定长帧用 `packed` 结构体 `peek()` 到局部变量，校验通过再赋给 `raw`；CRC 字段按小端 `uint16_t` 直接比较
- 只做协议规定的帧头、长度和 CRC 校验，字段原样映射，不额外检查浮点有限性或取值范围
- 断联只在 `update()` 检查：超时后 `is_connect = false` 并清零解析结果，不清 `rx_fifo` 和 `raw`
- 发送帧需要 SWD 诊断时保存为成员 `raw_tx`，`set_*()` 直接写入，`send()` 补 CRC 后发出
- CRC 为自由函数：`crc8_referee`、`crc16_referee`（`referee/crc.hpp`），`crc16_modbus`（`minipc/crc16_modbus.hpp`）

### 表达

- 多分支使用清晰的 `if`、`else if` 或提前返回，不使用嵌套三目
- 位运算的掩码和移位量使用无符号字面量，如 `1u << 0u`、`0xFFu`；按位表达式中的零值写 `0u`
- 参数使用对应物理量名称；与成员重名时用 `this->`
- 不会重新赋值的局部变量和按值参数在定义中尽量使用 `const`；声明中不重复顶层 `const`
- 指针和引用按实际修改语义限定
- 类成员不用 `std::array`，用 C 数组（EmbeddedProbe 无法解析类内 `std::array`）；函数参数与局部变量不限
- 遍历多个对象用指针列表（如 `{&w1, &w2}`），不用 `std::ref` 包装引用
- 内置整数等轻量类型默认后置 `i++`；需要新值或避免重载类型旧值副本时用前置 `++i`

### 类与文件

- static constexpr 数据常量放在 class 最前面，public 常量优先，其余按最小权限
- 类型按依赖声明；变量全部放在函数前，依次为：构造参数、set 输入、供 SWD 分析的状态、PID 组件、通常无需 SWD 分析的跨周期状态、Timer
- 构造参数按接口顺序排列，包含设备绑定、配置及由配置预计算的固定值；Pid 的配置指针属于构造参数，机构持有的 Pid 实例属于 PID 组件
- 独立的 set 输入放在观察状态前；已有 ref/measure 成对结构整体保留，作为目标与反馈观察组；状态机当前阶段放在观察状态，上一周期标记、累计量和内部缓存放在 PID 后
- 变量可穿插 public、protected、private；权限不改变观察顺序；当前无外部访问的使用 private
- 函数统一放在变量之后，依次为构造函数、set_enable、其他 set 函数、其他公开接口、update、内部辅助函数；send、查询等公开接口放在 set 组后、update 前；内部函数按入口首次调用顺序深度优先排列，调用者在前、被调用者紧随，共用函数按首次出现放置；保留最小访问权限，protected 构造同样前置，protected 辅助函数放在 update 后
- .cpp 和 .tpp 的函数定义按 .hpp 声明排序；构造初始化列表跟随成员声明顺序
- 结构体只有一个成员实例且类型与实例访问权限相同时，变量直接跟在声明后；公开构造类型与 private/protected 实例保持分开声明；复用的构造输入、协议载荷与返回类型按实际用途保留
- 电机实例不嵌套状态结构体；超级电容实例仅保留 RLS 嵌套组
- Config 内的字段按相似含义分组，组内紧邻，组间空行并用简短中文注释分隔
- 跨周期维护状态靠后，所有 Timer 成员集中在变量区最后，依次为其他 timer、callback_timer、update_timer；继承层各自按本层成员排序；固定绑定尽量 const，独立辅助运算尽量 static
- 创建时确定的结构体优先指定成员初始化，顺序与声明一致

### 单位

- 使用 SI 单位
- 通用单位换算放在 `math/unit.hpp`
- 协议刻度换算留在对应解析或发送处

## 注释与格式

- 注释精炼，优先用简短中文概括代码段职责；不直观处再补充配置、单位、边界、约束或原因，单句尽量不加句号
- 注释服务于上车前的逐行审计：读者对照需求逐段核对含义、单位、方向和条件，无需反推代码
- 变量与 Config 字段的行尾注释写“含义，单位；方向、条件或约定”，逗号接单位，分号接约定；同组注释由 clang-format 对齐
- 相关变量成组声明，组前注释写组名、计算位置和失效值；Config 组前注释写“组名：各字段含义，单位”，组内字段顺序与注释一致
- 主流程按输出对象或处理步骤分段，段间空行；段首注释写“对象：输入来源与映射规则；特殊条件”，按键、挡位、方向写出具体对应，优先级和互斥条件写明先后
- 示例（`lib/rc/vt13`）：

```cpp
// 平移：摇杆满量程与合成速度上限，m/s；键盘加速度、松手减速度，m/s^2
float vxy_max = 0;
float axy = 0, dxy = 0;

// 操控量，update() 计算，断联时为 0
bool is_enable = false; // N、S 挡使能
float wr = 0;           // 旋转角速度，rad/s；逆时针为正

// 云台：摇杆叠加鼠标，摇杆上推抬头；按住 trigger 时 pitch 摇杆改为调弹频
const float pitch_stick = rc.trigger ? 0 : rc.pitch;
yaw_speed = std::clamp((rc.yaw + mouse.yaw) * config.yaw_speed_max, -config.yaw_speed_max, config.yaw_speed_max);

// 弹频：滚轮由 0 上跳加一步、下跳减一步，限幅到 [min, max]
if (last_wheel == 0 && mouse.wheel > 0)
    config.bullet_freq = std::min(config.bullet_freq + config.bullet_freq_step, config.bullet_freq_max);
last_wheel = mouse.wheel;

// 发射：trigger 时 pitch 摇杆为弹频倍率，负值反转；否则鼠标左键按弹频
if (rc.trigger)
    bullet_freq = rc.pitch * config.bullet_freq;
```
- STM32 生成代码和第三方原始代码保持原样；生成文件中的应用修改放在 USER CODE 区域
- 遵循根 `.clang-format`，使用 `tools/format_code.py` 调用 CLion 同版本 clang-format
- 只格式化本任务修改的手写代码
- 函数与逻辑块合理留空行
- 头文件只包含自身必需的头；已被另一必需头文件包含的不重复包含
- Eigen 只经 `math/eigen.hpp` 包含，不直接包含 `<Eigen/...>`
- `.cpp` 先包含对应头文件，再连续包含 `<...>` 公共库，最后连续包含 `"..."` 项目库，三组之间不插空行
