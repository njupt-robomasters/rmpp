# attitude 示例 CPU 占用：Eigen 在 -Og 下变慢

## 现象（用户反馈）

- Debug（-Og）下 1 kHz 跑不完；Release（-O2）约 17%
- 换 Eigen 前手写 C 数组版本（`c9ac138^:lib/imu/quaternion_ekf.cpp`）在 -Og 下约 30%

## 静态分析（`compile_variants.sh`，只编译 `quaternion_ekf.cpp`）

| 选项 | .text | 函数数 | bl 调用 |
| --- | ---: | ---: | ---: |
| -Og | 66.7 KB | 579 | 1065 |
| -Og -DNDEBUG | 21.2 KB | 378 | 545 |
| -Og -DEIGEN_NO_DEBUG + EIGEN_STRONG_INLINE=always_inline | 21.1 KB | 95 | 204 |
| -O1 | 22.3 KB | 36 | 130 |
| -O2 | 17.7 KB | 31 | 102 |
| 旧手写版 -Og | 4.5 KB | — | 44 |

- -Og 不做普通内联（`-finline-*` 无效），Eigen 表达式模板的每层 evaluator/Block/MapBase 都成了真实函数调用；
  完全展开的赋值变成按下标递归的函数链（`copy_using_evaluator_LinearTraversal_CompleteUnrolling` 129 个实例）
- Debug 未定义 NDEBUG，`eigen_assert` 全部生效（-Og 下 140 处 `__assert_func`），代码量约 3 倍
- 旧手写版本是普通循环，-Og 下照样是紧凑代码，因此不受影响

## 上板测量（`app/` + `measure.sh`，C 板）

`app/` 前 10 s 只调 `imu.update()`，之后调 `attitude.update()`，DWT 计周期。`EKF_OPTIONS` 只给 `quaternion_ekf.cpp` 追加选项。
构建（输出在 `out/`）：

```sh
CM="/d/JetBrains/CLion 2026.2.2/bin/cmake/win/x64/bin/cmake.exe"
APP=../agents/tasks/08-attitude_cpu/app
"$CM" -S . -B agents/tasks/08-attitude_cpu/out/og -G Ninja -DRMPP_APP=$APP -DCMAKE_BUILD_TYPE=Debug
# og_nodebug: -DCMAKE_C_FLAGS=-DEIGEN_NO_DEBUG；og_ekf_o2: -DEKF_OPTIONS=-O2；o2: -DCMAKE_BUILD_TYPE=Release
sh agents/tasks/08-attitude_cpu/measure.sh og og_nodebug og_ekf_o2 o2
```

## 实测结果（2026-11-10，C 板 168 MHz，1 kHz 主循环）

| 变体 | imu.update() | attitude.update() | 其中 EKF | 主循环频率 | CPU |
| --- | ---: | ---: | ---: | ---: | ---: |
| og | 65 µs | 1354 µs | ≈1289 µs | 735 Hz | 99.7% |
| og_nodebug | 58 µs | 838 µs | ≈780 µs | 999 Hz | 83.8% |
| og_ekf_o2 | 65 µs | 198 µs | ≈133 µs | 999 Hz | 20.0% |
| o2 | 44 µs | 174 µs | ≈130 µs | 999 Hz | 17.5% |
| og_pragma | 65 µs | 241 µs | ≈175 µs | 999 Hz | 24.2% |

- EKF = attitude − imu（另含欧拉角转换，量很小）；imu 的 max 为首次初始化两次 80 ms 复位等待，忽略
- -Og 下 Eigen EKF 比 -O2 慢约 10 倍；关断言只快约 40%，仍慢 6 倍
- 只给 `quaternion_ekf.cpp` 加 -O2 即与全 O2 基本一致
- og_pragma：-Og，Eigen 头在 `#pragma GCC optimize("O2")` 内包含；solve() 自身仍为 -Og 可调试，只剩调用各 Eigen 顶层运算的开销（静态 bl 1065→370）

## 采用方案

- 用户选定 pragma 方案：`lib/math/eigen_types.hpp` 改名 `lib/math/eigen.hpp`，作为唯一 Eigen 入口在 pragma 区内包含 Cholesky、Geometry；其他文件不直接包含 `<Eigen/...>`
- 改后 `og` 变体即为该方案，实测 imu 54 µs、attitude 233 µs、CPU 23.4%（Imu 内的 Eigen 运算也随之变快）；板上留的是该固件
- 上表 og/og_nodebug/og_ekf_o2 为改动前数据，改动后重新构建将不再复现
