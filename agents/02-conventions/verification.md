# 验证与人工审计

用途：说明验证分层、常用命令、人工审计入口和板上注意事项。各任务的具体验证结果和复现命令在对应的 `agents/<主题>/` 记录中，本文不重复。

## 验证分层

- 格式检查：只覆盖本任务改动的手写文件，使用与 CLion 同版本的 clang-format
- 局部编译：相关 `.cpp` 的 ARM 对象编译，借用 `build/<目标>/compile_commands.json`，不链接固件
- 完整构建：用户要求时执行，报告错误、警告和产物大小
- 主机测试：编译真实业务代码，用桩替代硬件，覆盖协议、状态机、功率和类布局
- 硬件验证：烧录、台架、上车，由用户审计后决定；报告时说清实际进行的是哪一种

## 常用命令

```sh
python tools/update_presets.py
python tools/check_builds.py                 # 批量构建，日志在 build/checks/
python tools/format_code.py --clang-format <路径> --check
```

- 目标应用用 `cmake --preset <名称>` 配置，产物为 `build/<preset>/rmpp.elf` 和 `rmpp.map`
- 主机测试按任务记录中的命令单独编译运行，测试可执行文件和日志写到 `agents/tmp/<任务名>/`
- 需要指定工具链时用 `--cmake`、`--ninja`、`--clang-format` 传 CLion 自带版本

## 人工审计入口

1. `lib/bsp/`：公共接口、无效句柄、`Bsp` 入口
2. 板卡 `bsp/`：构造绑定、metrics、临界区、收发与 PWM/SPI 实现
3. app：硬件绑定、控制周期与发送顺序、模块间数据传递
4. `lib/robot/`、`lib/motor/`、`lib/power/`：控制律、限幅、方向、单位和功率口径
5. 根、app、lib、BSP 的 CMake 与 `.clang-format`

## 板上验证注意

- 软件验证不覆盖真实中断耗时、总线负载、USB 枚举、PWM 波形和机械行为
- CAN 发送成功只表示提交邮箱；周期发送在忙时返回失败，下个周期发新值
- UART 在通知订阅者后重新接收，需检查实际回调耗时与接收间隙
- metrics 仅在收到报文时更新，停收保留最后结果
- 烧录、复位、halt、改写内存或寄存器前先说明操作并经用户同意；用户单板测试常不接电源，板子在机器人上时擅自操作可能使机构误动作
