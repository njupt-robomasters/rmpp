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
g++ -std=c++23 -O2 -Wall -Wextra -I../../../lib -I../../../lib/math sim.cpp ../../../lib/power/super_cap.cpp -o out/sim.exe && ./out/sim.exe
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
g++ -std=c++23 -O2 -I../../../lib -I../../../lib/math power_sim.cpp ../../../lib/power/super_cap.cpp -o out/power_sim.exe && ./out/power_sim.exe
```

结论（60 W 裁判上限，需求 200 W，OFF → CHARGE → BOOST → CHARGE → OFF）：

- CHARGE 稳态底盘 60 W、电容功率 0，不放电；电池 60 W，裁判缓冲能量稳在 57.3 J
- 模型低估 20%：CHARGE 稳态 `energy_local` 降到 51.8 J，底盘真实功率仍为 60 W、电容功率约 0；偏差由积分吃掉
- BOOST 满额补偿约 140 W，电压降到 12 V 以下后补偿线性衰减；电容将空时 chassis_ref 平滑回落到 60 W
- 切换无能量跳变；`chassis_ref` 的大跳变只在切入切出 BOOST 时，来自 boost_max 本身
