"""编译运行 power_test.cpp，用仿真 lab.limit_wheel_power() 复算每个工况的功率分配并对照

- 固件口径：腿部映射按 LQR_FIT 与 LQR_POSTURE_FIT 拟合表求，应与固件一致到浮点误差
- 仿真映射：腿部映射取 lab.power_maps()（精确求解两腿稳姿态 LQR），差异来自拟合误差
用法：python check_power.py（需 wheel_leg_lab 的 .venv 与 MinGW g++）
"""
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / 'agents/tmp/power_alloc'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT.parent / 'wheel_leg_lab'))
from check_maps import firmware_maps  # noqa: E402
from lab import Settings, limit_wheel_power, power_maps  # noqa: E402

WHEEL_MAX = 20 * 0.2545043441
JOINT_MAX = 40.
LENGTH = .18
FIELDS = {'omega': 2, 'budget': 1, 'raw': 4, 'limited': 4, 'axial': 2}


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / 'power_test.exe'
    sources = ['lib/robot/chassis_leg.cpp', 'lib/motor/motor.cpp', 'lib/motor/m3508.cpp', 'lib/pid/pid.cpp']
    command = ['g++', '-std=c++20', '-O1', '-mno-ms-bitfields', f'-I{HERE / "stubs"}', f'-I{ROOT / "lib"}', f'-I{ROOT / "lib/third_party/eigen-3.4.0"}',
               str(HERE / 'power_test.cpp'), *(str(ROOT / s) for s in sources), '-o', str(exe)]
    subprocess.run(command, check=True)
    return exe


def parse(line):
    name, *values = line.split()
    field, i = {}, 0
    while i < len(values):
        size = FIELDS[values[i]]
        field[values[i]] = np.array([float(v) for v in values[i + 1:i + 1 + size]])
        i += 1 + size
    return name, field


def main():
    result = subprocess.run([str(build())], capture_output=True)
    print(result.stderr.decode('utf-8').strip())
    if result.returncode:
        sys.exit(result.returncode)
    firmware = firmware_maps(LENGTH, LENGTH)[:2]
    settings = Settings(length=LENGTH)
    lab = power_maps(settings, settings.gain_si())
    worst_firmware = worst_lab = 0.
    for line in result.stdout.decode('utf-8').splitlines():
        name, field = parse(line)
        omega, budget, raw, limited, axial = field['omega'], field['budget'][0], field['raw'], field['limited'], field['axial']
        swing = 2 * np.maximum(JOINT_MAX - abs(axial), 0)
        bounds = (np.r_[-WHEEL_MAX, -WHEEL_MAX, -swing], np.r_[WHEEL_MAX, WHEEL_MAX, swing])
        expect = limit_wheel_power(raw, omega, budget, bounds, firmware)
        expect_lab = limit_wheel_power(raw, omega, budget, bounds, lab)
        error, error_lab = np.max(abs(limited - expect)), np.max(abs(limited - expect_lab))
        worst_firmware, worst_lab = max(worst_firmware, error), max(worst_lab, error_lab)
        print(f'{name}: 功率 {raw[:2] @ omega:7.2f} → {limited[:2] @ omega:7.2f} W（预算 {budget:g}）'
              f'  轮 {np.array2string(raw[:2], precision=3)} → {np.array2string(limited[:2], precision=3)}'
              f'  腿 {np.array2string(raw[2:], precision=3)} → {np.array2string(limited[2:], precision=3)} N·m'
              f'  差：固件口径 {error:.1e}，仿真映射 {error_lab:.3f}')
    print(f'最大差：固件口径 {worst_firmware:.2e} N·m，仿真映射 {worst_lab:.3f} N·m')
    if worst_firmware > 2e-3:
        sys.exit('FAIL: 固件与仿真算法不一致')


if __name__ == '__main__':
    main()
