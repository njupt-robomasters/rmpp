# 项目与协作约定

RMPP 是 STM32 机器人控制项目。入门见 `README.md`；目录、接口、构建见 `docs/architecture.md`；验证与审计见 `docs/verification.md`；单位约定见 `docs/unit-migration.md`。

## 编码

- 简洁、直观、可人工审计优先。清理重复实现、无效中间层、旧接口与依赖；统一同类代码的写法，以编译和测试验证简单实现。
- 手写代码使用全局作用域，文件私有函数和变量用 static；第三方保留原有组织。类型 PascalCase，函数、变量、文件 snake_case，私有及受保护成员尾随 `_`，常量和枚举 UPPER_SNAKE_CASE；单位符号、型号保留领域惯例。
- 创建时确定的结构体优先指定成员初始化，成员顺序与声明一致。单位换算在 `math/unit.hpp`。
- 排版以根 `.clang-format` 为准：LLVM、四空格、同行大括号、无列宽限制；函数与逻辑块留空行。修改后用与 CLion 相同的 clang-format 执行 `tools/format_code.py`；第三方与生成代码保留原风格。
- 外设同时支持 `bsp.can(1)` 与 `Can(1)`，整数 ID 可不连续。Bsp 只构造句柄，行为由各外设类实现。无效句柄可判 bool，操作安全失败。调试数据位于通道状态的 metrics，业务通过操作返回值判断结果。
- 同通道句柄共享状态；Timer 独立计时。全局构造只处理软件状态；板卡 `bsp_init()` 在 CubeMX 初始化后启动外设，随后执行 `app_main()`。
- 接收使用单参数 `add_rx_callback(std::function)`，每通道 vector，中断内依序通知全部订阅者。CAN 直接提交邮箱并返回结果。临界区直接保存、关闭、恢复 PRIMASK。
- app 选择硬件并向 lib 传句柄。RGB 逻辑在 lib，SPI、Storage 和 FlashDB 实现在 BSP。手写源码用 `GLOB_RECURSE CONFIGURE_DEPENDS "*.*"`；CubeMX 默认 CMake 由 BSP 接入。

## 协作与交付

- 已确认需求直接推进，已有授权持续有效；必要问题一次说明原因。
- 先核对实际代码和配置，保护工作区修改；按要求保留重构前 Git 基线。
- 先说明结果，结合代码、真实编译和测试说明变化及限制；用户人工审计后决定上车测试。
- 本文件只保存稳定约定；细节写入简洁有序的中文 docs。更新最终结论并清理失效内容，讨论过程留在对话中。
