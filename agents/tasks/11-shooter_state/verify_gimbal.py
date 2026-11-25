"""比较独立云台与指定基线，真实编译云台、Motor、PID，仅模拟反馈和时钟。"""

import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = Path(__file__).resolve().parents[2] / "out/11-shooter_state/gimbal_standalone"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--baseline-root", type=Path)
    parser.add_argument("--baseline-ref", default="f3b8e14")
    args = parser.parse_args()
    baseline = OUT / "baseline"
    baseline.mkdir(parents=True, exist_ok=True)
    for stem in ["gimbal", "gimbal_classic", "gimbal_dual_yaw"]:
        for suffix in [".hpp", ".cpp"]:
            relative = f"lib/robot/gimbal/{stem}{suffix}"
            if args.baseline_root:
                text = (args.baseline_root / relative).read_text(encoding="utf-8")
            else:
                text = subprocess.check_output(
                    ["git", "show", f"{args.baseline_ref}:{relative}"], cwd=ROOT
                ).decode("utf-8")
            text = text.replace("Gimbal", "BaselineGimbal")
            (baseline / (stem + suffix)).write_text(text, encoding="utf-8")
    sources = [TASK / "gimbal_standalone_test.cpp", *baseline.glob("*.cpp")]
    sources += [ROOT / "lib" / unit for unit in [
        "robot/gimbal/gimbal_classic.cpp", "robot/gimbal/gimbal_dual_yaw.cpp",
        "motor/motor.cpp", "pid/pid.cpp",
    ]]
    exe = OUT / "gimbal_standalone_test.exe"
    subprocess.run([
        args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra",
        "-I" + str(OUT), "-I" + str(ROOT / "lib"),
        "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
        *map(str, sources), "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
