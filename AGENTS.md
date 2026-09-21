# 项目与协作约定

RMPP 是 STM32 机器人控制项目。快速入门见 `README`；目录、接口、构建见 `docs/architecture.md`；验证与审计见 `docs/verification.md`；单位约定见 `docs/unit-migration.md`。

## 编码

- 简洁、直观、可人工审计优先。抽象、状态和初始化阶段必须有具体用途；优先验证简单实现。
- 手写类型用 PascalCase，函数、变量、文件用 snake_case，私有及受保护数据成员尾随 `_`，常量和枚举值用 UPPER_SNAKE_CASE；单位符号和型号保留领域惯例。
- 排版以根目录 `.clang-format` 为准：LLVM、四空格、同行大括号、无列宽限制。生成及第三方代码保持原风格。
- 使用 `auto can = board.can(1)`；整数 ID 可不连续。Board 只查找资源，逻辑属于对应外设类。空句柄可判 bool，各 API 空操作并返回失败。
- 外设句柄按值共享通道状态；Timer 独立计时。全局构造只处理软件状态；板卡 `bsp_init()` 在 CubeMX 初始化后启动外设，随后进入 `app_main()`。
- 接收用单参数 `add_rx_callback(std::function)`，每通道 vector，中断内依序通知全部订阅者。CAN 直接调用 HAL 添加邮箱，返回提交结果。
- app、lib、BSP 各自组织编译；手写源码用 `GLOB_RECURSE CONFIGURE_DEPENDS "*.*"`。CubeMX 默认 CMake 由板卡 BSP 接入。

## 协作与交付

- 已确认的需求直接推进，已有授权持续有效；必要问题一次说明原因。
- 先核对现有代码和配置，保护工作区修改；重构前按用户要求保存 Git 快照。
- 先说结果，结合具体代码、真实编译和测试说明改动及限制。发现问题继续修正，交付便于人工审计。
- 用户人工审计后决定上车测试；构建通过只说明软件检查通过。
- 本文件只保留稳定约定；细节写入中文 docs，每篇开头说明用途。更新最终结论、清理失效和重复内容，讨论过程留在对话中。
