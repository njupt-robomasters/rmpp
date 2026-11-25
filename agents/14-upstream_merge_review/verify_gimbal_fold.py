"""在相同反馈和时钟下，对比 app 内的折叠云台与迁移前实现。"""

import argparse
import re
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/14-upstream_merge_review/gimbal_fold"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", nargs="+", default=["g++"])
    args = parser.parse_args()
    baseline = OUT / "baseline"
    baseline.mkdir(parents=True, exist_ok=True)
    for name in ("gimbal.hpp", "gimbal.cpp", "gimbal_fold.hpp", "gimbal_fold.cpp"):
        source = subprocess.check_output(
            ["git", "show", f"a220339:lib/robot/{name}"], cwd=ROOT
        ).decode("utf-8")
        source = re.sub(r"\.position\b", ".angle", source.replace("Gimbal", "BaselineGimbal"))
        # 历史 Motor 的 position 离线保留；新 Motor 的 angle 超时归零，使用既有轴测量恢复旧反馈约定
        source = re.sub(r"(\s+)([\w.]+\.measure) = (motor\.\w+)\.angle;",
                        r"\1if (\3.is_connect)\1    \2 = \3.angle;", source)
        (baseline / name).write_text(source, encoding="utf-8")
    exe = OUT / "gimbal_fold_test.exe"
    subprocess.run([
        *args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra",
        "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
        "-I" + str(OUT), "-I" + str(ROOT / "lib"),
        "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
        "-I" + str(ROOT / "app/jelly_sentry"),
        str(TASK / "gimbal_fold_test.cpp"), *map(str, sorted(baseline.glob("*.cpp"))),
        str(ROOT / "lib/robot/gimbal_fold.cpp"),
        str(ROOT / "lib/motor/motor.cpp"), str(ROOT / "lib/pid/pid.cpp"),
        "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
