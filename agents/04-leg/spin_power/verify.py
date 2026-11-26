"""主机侧编译并运行轮端功率限幅回归，输出目录 agents/tmp/spin_power。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/spin_power"

SOURCES = [
    TASK / "test_limit_power.cpp",
    *(ROOT / p for p in ["lib/motor/motor.cpp", "lib/motor/m3508.cpp", "lib/pid/pid.cpp", "lib/power/motor_power.cpp", "lib/robot/chassis_leg.cpp"]),
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / "test_limit_power.exe"
    command = [
        "g++", "-std=c++23", "-O2", "-Wall", "-Wextra",
        "-I" + str(TASK / "stubs"),
        "-I" + str(ROOT / "app/leg/leg_down"),
        "-I" + str(ROOT / "lib"),
        "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
        *map(str, SOURCES),
        "-o", str(exe),
    ]
    subprocess.run(command, check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
