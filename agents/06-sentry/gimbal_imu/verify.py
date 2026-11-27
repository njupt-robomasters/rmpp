"""编译真实云台，对照合并前 pitch2 的逐拍输出；产物放在 agents/tmp/gimbal_imu。"""

import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/gimbal_imu"
BASELINE = "f3b6c31a516cf73a87a50ad803cad4f1284fe28d"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for suffix in ("hpp", "cpp"):
        source = subprocess.check_output(
            ["git", "show", f"{BASELINE}:lib/robot/gimbal_fold.{suffix}"], cwd=ROOT
        ).decode("utf-8")
        source = source.replace("GimbalFold", "BaselineGimbalFold")
        source = source.replace('"gimbal_fold.hpp"', '"baseline_gimbal_fold.hpp"')
        (OUT / f"baseline_gimbal_fold.{suffix}").write_text(source, encoding="utf-8")
    env = {**os.environ, "ZIG_GLOBAL_CACHE_DIR": str(OUT / "zig_cache"),
           "ZIG_LOCAL_CACHE_DIR": str(OUT / "zig_local_cache")}
    common = [str(ROOT / "agents/.tools/ziglang/zig.exe"), "c++", "-std=c++23", "-O1",
              "-Wall", "-Wextra", "-Wno-unknown-pragmas", "-Wno-deprecated-declarations",
              "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
              "-Ilib", "-isystem", "lib/third_party/eigen-3.4.0", "-I" + str(OUT)]
    for name in ("pitch_regression",):
        exe = OUT / f"{name}.exe"
        units = [str(TASK / f"{name}.cpp"), "lib/robot/gimbal_fold.cpp",
                 "lib/motor/motor.cpp", "lib/pid/pid.cpp"]
        if name == "pitch_regression":
            units.append(str(OUT / "baseline_gimbal_fold.cpp"))
        result = subprocess.run([*common, *units, "-o", str(exe)], cwd=ROOT, env=env,
                                capture_output=True, text=True, encoding="utf-8", errors="replace")
        (OUT / f"{name}_compile.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            print(result.stdout + result.stderr, flush=True)
        result.check_returncode()
        result = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True,
                                encoding="utf-8", errors="replace")
        (OUT / f"{name}.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout + result.stderr, end="", flush=True)
        result.check_returncode()


if __name__ == "__main__":
    main()
