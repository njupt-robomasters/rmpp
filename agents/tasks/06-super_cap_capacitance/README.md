# 超级电容在线容量辨识

实现在 `lib/power/super_cap.{hpp,cpp}`，结果 `SuperCap::capacitance`（F）、`SuperCap::resistance`（ESR，Ω）供 SWD 观察

## 算法

- 模型：`V_CAP = Q/C + ESR·I_CAP`，电流充电为正
- 在 `can_callback` 调用的 `update_capacitance` 中用原始（未低通）`I_CAP` 梯形积分电荷；窗口内电荷达 `Config::rls.min_charge`（默认 1 C）时做一步二参数 RLS：
  `ΔV = ΔQ·(1/C) + ΔI·ESR`，差分消去未知的初始电荷
- 窗口超过 `rls.max_window`（默认 1 s）未达电荷则重新开窗，即平均电流须大于 1 A 才更新，静置时电流零偏不会拖偏结果
- 首帧、应答间隔超过 `timeout` 时重新开窗
- 遗忘因子 `rls.lambda`（默认 0.98）仅在协方差迹小于初值时使用，防止 ΔI 长期为零（恒流充电）时 ESR 方向协方差发散
- `1/C ≤ 0` 时输出零，代表无效

## 主机仿真

`sim.cpp` 用 RC 模型生成 0x051 应答（100 Hz，充电 3 A / 放电 −15 A 脉冲 / 静置），桥接 Can 与 Timer 后链接真实 `super_cap.cpp`

```sh
cd agents/tasks/06-super_cap_capacitance
g++ -std=c++23 -O2 -Wall -Wextra -I../../../lib -I../../../lib/math sim.cpp ../../../lib/power/super_cap.cpp -o ../../out/06-super_cap_capacitance/sim.exe && ../../out/06-super_cap_capacitance/sim.exe
```

结论（10 s 内收敛）：

| 工况 | 真值 | 估计 |
| --- | --- | --- |
| 理想 | 6 F / 0.1 Ω | 5.99–6.00 F / 0.101 Ω |
| 0.1 A 噪声 | 6 F / 0.1 Ω | 5.98–6.01 F / 0.101 Ω |
| 0.1 A 噪声 + 0.1 A 偏置 | 6 F / 0.1 Ω | 6.02–6.10 F / 0.101 Ω |
| 小容量 | 2 F / 0.2 Ω | 2.00 F / 0.203 Ω |

电流偏置引入约 1% 容量偏差；应答间隔可忽略的 ESR 偏差来自梯形积分

## 待上车核对

- `I_CAP` 正负与 `V_CAP` 变化方向一致（充电时电压上升），否则 `capacitance` 始终为零
- CHARGE 恒功率充电时 ΔI 小，ESR 收敛慢；BOOST 放电脉冲后观察是否稳定
- DCDC 损耗不影响辨识（只用电容侧 V、I）；应答量程 int16 mA 上限 32 A

## 虚拟缓冲能量功率控制

`power_sim.cpp`：底盘固定需求、按 `chassis_ref` 限幅（可设模型低估），超电 DCDC 把电池侧稳在 `battery_ref`，电容 8 V 止放、满电停充，裁判 10 Hz 结算

```sh
g++ -std=c++23 -O2 -I../../../lib -I../../../lib/math power_sim.cpp ../../../lib/power/super_cap.cpp -o ../../out/06-super_cap_capacitance/power_sim.exe && ../../out/06-super_cap_capacitance/power_sim.exe
```

结论（60 W 裁判上限，需求 200 W，OFF → CHARGE → BOOST → CHARGE → OFF）：

- CHARGE 稳态底盘 60 W、电容功率 0，不放电；电池 60 W，裁判缓冲能量稳在 57.3 J
- 模型低估 20%：CHARGE 稳态 `energy_local` 降到 51.8 J，底盘真实功率仍为 60 W、电容功率约 0；偏差由积分吃掉
- BOOST 满额补偿约 140 W，电压降到 12 V 以下后补偿线性衰减；电容将空时 chassis_ref 平滑回落到 60 W
- 切换无能量跳变；`chassis_ref` 的大跳变只在切入切出 BOOST 时，来自 boost_max 本身

## jelly_sentry 接入（2026-11-16）

用户确认超电已装在 CAN1，ID 保持默认。整车复用原有 SuperCap 和四轮力控链路，补充模式选择、100 Hz 持续发送；底盘调试入口提供独立 `cap_mode`，默认 OFF，固定裁判输入 120 W/60 J。模式与裁判失联处理见 `agents/docs/application.md` 的“jelly_sentry 超级电容”

### 验证与复现

- Arm GCC 14.3.1：整车 `app.cpp`、`chassis.cpp` 与底盘调试入口 `app.cpp` 对象局部编译通过，无编译警告
- clang-format 22.1.0：通过 `tools/format_code.py` 格式化本次四个手写文件并检查；`git diff --check` 核对本次范围
- `verify_jelly_sentry.py` 提取实际 C++ 模式条件并在主机求值，与独立策略表比较 750 种输入组合，覆盖两个遥控器并存、显式关闭、无效档位、裁判离线及残留比赛阶段、红蓝哨兵与其他机器人 ID
- 模式检查属于条件求值检查；硬件时序、真实应答和供电行为待台架验证。本轮未完整链接或操作硬件

```powershell
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/tasks/06-super_cap_capacitance/verify_jelly_sentry.py
& 'C:/Users/joushua/AppData/Local/stm32cube/bundles/cmake/4.4.0+st.1/bin/cmake.exe' --build build/jelly_sentry --target app/CMakeFiles/rmpp_app.dir/app.cpp.obj app/CMakeFiles/rmpp_app.dir/chassis.cpp.obj -j 4
& 'C:/Users/joushua/AppData/Local/stm32cube/bundles/cmake/4.4.0+st.1/bin/cmake.exe' --build build/jelly_sentry_debug-chassis --target app/CMakeFiles/rmpp_app.dir/app.cpp.obj -j 4
```

### 用户审计后的台架步骤

1. 使用底盘调试入口，保持 `is_enable=false`、`cap_mode=0`：观察超电反馈与功率，核对 0x061/0x051 及约 10 ms 请求/应答间隔
2. 将 `cap_mode` 设为 1（CHARGE）：核对电容电压上升、电容充电功率为正；满电或负载变化时结合工况判断
3. 在用户确认的底盘负载下设为 2（BOOST）：观察 `power_chassis_ref` 提升、电容放电功率为负，以及 12 V 至 10 V 的补偿衰减
4. 检查超电反馈超时后取消 BOOST 补偿、重连恢复；整车再检查遥控显式 OFF、比赛阶段切换、遥控断联自主运行和裁判断联转 OFF
5. 核对超电采样覆盖的供电支路，结合四轮 `power.estimate` 分析功率差异；实际 CAN 周期、稳定性和充放电量程均以板上结果记录

### VT13 C/N 挡（2026-11-18）

- 在线 C 挡优先使整车失能。N/S 挡请求 BOOST，受裁判失联和人工 OFF 保护，N 挡双传送带与拨盘目标归零
- 整车 `rmpp_app` 局部编译和 750 项超电模式检查通过。充放电行为待台架验证

- 用户补充 rub1/rub2 用于工业传送带。整车接通两路 VESC 发送，S 挡给速度目标，按住扳机后左摇杆上下给拨盘目标。2026-11-18 联调时用户确认只使用扳机与左摇杆。实现入口为 `app/jelly_sentry/shooter.cpp`，报文发送在 `app.cpp`
